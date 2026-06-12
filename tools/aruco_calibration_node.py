#!/usr/bin/env python3
"""
ArUco-based hand-eye calibration for the FR3WML setup (eye-to-hand: camera
mounted on the robot base, marker placed on the table).

Procedure
---------
1. Robot must be in drag-teach (motors disabled / compliant mode).
2. Touch 3 corners of the ArUco BLACK SQUARE with the suction-cup tip,
   in order: top-left (TL), top-right (TR), bottom-left (BL).
   The node reads tf2 base_link -> wrist3_link at each touch and applies
   the tool Z offset (default 0.190 m) to compute the cup tip position
   in base_link. From these 3 points it builds T_base_marker_center.
3. Move robot out of the camera FOV. The node acquires N camera frames,
   detects the marker, estimates its pose in camera_color_optical_frame
   via solvePnP, and averages across frames.
4. Composes the chain:
       T_base_camera_optical = T_base_marker_center @ inv(T_camera_optical_marker)
       T_base_camera_link    = T_base_camera_optical @ inv(T_link_optical)
   where T_link_optical is read live from tf2 (published by the RealSense
   driver). The first chain step removes the marker as a reference; the
   second converts from optical (REP-103 z-forward) to mechanical
   (camera_link) so the result is ready to paste into the URDF.
5. Prints the URDF snippet for the camera_joint and saves a YAML log.

Usage
-----
    source ~/fr3wml_ws/install/setup.bash
    python3 src/fr3wml_industrial_bt/tools/aruco_calibration_node.py \\
        --ros-args \\
        -p marker_size:=0.090 \\
        -p marker_id:=0 \\
        -p tool_z_offset:=0.190

Topics needed
-------------
    /camera/color/image_raw            (sensor_msgs/Image)
    /camera/color/camera_info          (sensor_msgs/CameraInfo)
    /tf, /tf_static                    (must include base_link, wrist3_link,
                                        camera_link, camera_color_optical_frame)

Deps
----
    pip install opencv-contrib-python numpy pyyaml
    sudo apt install ros-humble-cv-bridge
"""

import sys
import threading
import time
from datetime import datetime
from pathlib import Path

import cv2
import numpy as np
import yaml

import rclpy
import tf2_ros
from cv_bridge import CvBridge
from geometry_msgs.msg import TransformStamped
from rclpy.duration import Duration
from rclpy.node import Node
from rclpy.qos import (
    qos_profile_sensor_data,
    QoSProfile,
    QoSReliabilityPolicy,
    QoSDurabilityPolicy,
    QoSHistoryPolicy,
)
from rclpy.time import Time
from sensor_msgs.msg import CameraInfo, Image


# ───────────────────────────────────────────────────────────────────── math helpers
def quaternion_matrix(q):
    """Quaternion (x, y, z, w) -> 4x4 rotation matrix."""
    x, y, z, w = q
    n = x * x + y * y + z * z + w * w
    if n < 1e-12:
        return np.eye(4)
    s = 2.0 / n
    wx, wy, wz = s * w * x, s * w * y, s * w * z
    xx, xy, xz = s * x * x, s * x * y, s * x * z
    yy, yz, zz = s * y * y, s * y * z, s * z * z
    M = np.eye(4)
    M[0, 0] = 1.0 - (yy + zz); M[0, 1] = xy - wz;       M[0, 2] = xz + wy
    M[1, 0] = xy + wz;         M[1, 1] = 1.0 - (xx + zz); M[1, 2] = yz - wx
    M[2, 0] = xz - wy;         M[2, 1] = yz + wx;       M[2, 2] = 1.0 - (xx + yy)
    return M


def quaternion_from_matrix(M):
    """4x4 (or 3x3) matrix -> quaternion (x, y, z, w)."""
    R = M[:3, :3]
    tr = R[0, 0] + R[1, 1] + R[2, 2]
    if tr > 0:
        S = 2.0 * np.sqrt(tr + 1.0)
        w = 0.25 * S
        x = (R[2, 1] - R[1, 2]) / S
        y = (R[0, 2] - R[2, 0]) / S
        z = (R[1, 0] - R[0, 1]) / S
    elif (R[0, 0] > R[1, 1]) and (R[0, 0] > R[2, 2]):
        S = 2.0 * np.sqrt(1.0 + R[0, 0] - R[1, 1] - R[2, 2])
        w = (R[2, 1] - R[1, 2]) / S
        x = 0.25 * S
        y = (R[0, 1] + R[1, 0]) / S
        z = (R[0, 2] + R[2, 0]) / S
    elif R[1, 1] > R[2, 2]:
        S = 2.0 * np.sqrt(1.0 + R[1, 1] - R[0, 0] - R[2, 2])
        w = (R[0, 2] - R[2, 0]) / S
        x = (R[0, 1] + R[1, 0]) / S
        y = 0.25 * S
        z = (R[1, 2] + R[2, 1]) / S
    else:
        S = 2.0 * np.sqrt(1.0 + R[2, 2] - R[0, 0] - R[1, 1])
        w = (R[1, 0] - R[0, 1]) / S
        x = (R[0, 2] + R[2, 0]) / S
        y = (R[1, 2] + R[2, 1]) / S
        z = 0.25 * S
    return np.array([x, y, z, w])


def euler_from_matrix(M):
    """4x4 (or 3x3) matrix -> (roll, pitch, yaw) URDF (Rz·Ry·Rx) convention."""
    R = M[:3, :3]
    sy = -R[2, 0]
    if abs(sy) < 1.0 - 1e-9:
        pitch = np.arcsin(sy)
        roll = np.arctan2(R[2, 1], R[2, 2])
        yaw = np.arctan2(R[1, 0], R[0, 0])
    else:  # gimbal lock
        pitch = np.copysign(np.pi / 2, sy)
        roll = np.arctan2(-R[1, 2], R[1, 1])
        yaw = 0.0
    return roll, pitch, yaw


def euler_matrix(roll, pitch, yaw):
    """URDF rpy -> 4x4. R = Rz(yaw) · Ry(pitch) · Rx(roll)."""
    cr, sr = np.cos(roll), np.sin(roll)
    cp, sp = np.cos(pitch), np.sin(pitch)
    cy, sy = np.cos(yaw), np.sin(yaw)
    Rx = np.array([[1, 0, 0], [0, cr, -sr], [0, sr, cr]])
    Ry = np.array([[cp, 0, sp], [0, 1, 0], [-sp, 0, cp]])
    Rz = np.array([[cy, -sy, 0], [sy, cy, 0], [0, 0, 1]])
    M = np.eye(4)
    M[:3, :3] = Rz @ Ry @ Rx
    return M


def to_homog(R, t):
    T = np.eye(4)
    T[:3, :3] = R
    T[:3, 3] = np.asarray(t).flatten()
    return T


def transform_stamped_to_matrix(t: TransformStamped) -> np.ndarray:
    q = [t.transform.rotation.x, t.transform.rotation.y,
         t.transform.rotation.z, t.transform.rotation.w]
    p = [t.transform.translation.x, t.transform.translation.y,
         t.transform.translation.z]
    T = quaternion_matrix(q)
    T[:3, 3] = p
    return T


def matrix_to_xyz_rpy(T):
    xyz = T[:3, 3].copy()
    rpy = euler_from_matrix(T)
    return xyz, rpy


def average_poses(positions, quaternions):
    """Mean translation (linear) + mean quaternion (sign-aligned + normalize)."""
    pos = np.mean(np.stack(positions, axis=0), axis=0)
    Q = np.stack(quaternions, axis=0)  # (N, 4) as (x, y, z, w)
    ref = Q[0]
    signs = np.sign(Q @ ref)
    signs[signs == 0] = 1.0
    Q = Q * signs[:, None]
    q_mean = np.mean(Q, axis=0)
    q_mean = q_mean / np.linalg.norm(q_mean)
    return pos, q_mean


# ─────────────────────────────────────────────────────────────────────── node
class ArucoCalibrationNode(Node):
    def __init__(self):
        super().__init__('aruco_calibration_node')

        self.declare_parameter('marker_size', 0.090)
        self.declare_parameter('marker_id', 0)
        self.declare_parameter('tool_z_offset', 0.190)
        self.declare_parameter('num_samples', 50)
        self.declare_parameter('image_topic', '/camera/color/image_raw')
        self.declare_parameter('camera_info_topic', '/camera/color/camera_info')
        self.declare_parameter('base_frame', 'base_link')
        self.declare_parameter('wrist_frame', 'wrist3_link')
        self.declare_parameter('camera_link_frame', 'camera_link')
        self.declare_parameter('camera_optical_frame', 'camera_color_optical_frame')
        self.declare_parameter('aruco_dict', 'DICT_5X5_50')
        self.declare_parameter('output_dir', '')

        gp = self.get_parameter
        self.marker_size = float(gp('marker_size').value)
        self.marker_id = int(gp('marker_id').value)
        self.tool_z_offset = float(gp('tool_z_offset').value)
        self.num_samples = int(gp('num_samples').value)
        self.image_topic = gp('image_topic').value
        self.camera_info_topic = gp('camera_info_topic').value
        self.base_frame = gp('base_frame').value
        self.wrist_frame = gp('wrist_frame').value
        self.camera_link_frame = gp('camera_link_frame').value
        self.camera_optical_frame = gp('camera_optical_frame').value
        self.aruco_dict_name = gp('aruco_dict').value
        self.output_dir = gp('output_dir').value

        # tf2 (spin_thread=True is the default → buffer fills in the background)
        self.tf_buffer = tf2_ros.Buffer()
        self.tf_listener = tf2_ros.TransformListener(self.tf_buffer, self)

        # camera state
        self.bridge = CvBridge()
        self.cam_K = None
        self.cam_D = None
        self.frame_lock = threading.Lock()
        self.latest_frame = None
        self.image_sub = None
        self.info_sub = None

        # ArUco dictionary + detector (new API if available, legacy otherwise)
        dict_name = self.aruco_dict_name.upper()
        if not hasattr(cv2.aruco, dict_name):
            self.get_logger().fatal(f"Unknown ArUco dictionary: {dict_name}")
            sys.exit(1)
        self.aruco_dict = cv2.aruco.getPredefinedDictionary(getattr(cv2.aruco, dict_name))
        if hasattr(cv2.aruco, 'ArucoDetector'):
            params = cv2.aruco.DetectorParameters()
            self.aruco_detector = cv2.aruco.ArucoDetector(self.aruco_dict, params)
            self.use_new_api = True
        else:
            self.aruco_detector = None
            self.aruco_params = cv2.aruco.DetectorParameters_create()
            self.use_new_api = False

        self.pnp_flag = getattr(cv2, 'SOLVEPNP_IPPE_SQUARE', cv2.SOLVEPNP_ITERATIVE)

        self.get_logger().info(
            f"ready  marker_size={self.marker_size*1000:.1f} mm  marker_id={self.marker_id}  "
            f"tool_z_offset={self.tool_z_offset*1000:.1f} mm  dict={self.aruco_dict_name}  "
            f"samples={self.num_samples}"
        )

    # ─────────────────────────────────────────────────────────── touch sequence
    def lookup_T_base_wrist(self) -> np.ndarray:
        try:
            t = self.tf_buffer.lookup_transform(
                self.base_frame, self.wrist_frame,
                Time(), timeout=Duration(seconds=2.0))
        except Exception as ex:
            raise RuntimeError(
                f"tf2 {self.base_frame} -> {self.wrist_frame} lookup failed: {ex}"
            )
        return transform_stamped_to_matrix(t)

    def compute_tip_in_base(self, T_base_wrist3: np.ndarray) -> np.ndarray:
        tool_z = T_base_wrist3[:3, 2]                  # wrist3 Z in base frame
        return T_base_wrist3[:3, 3] + self.tool_z_offset * tool_z

    def capture_corner(self, name: str) -> np.ndarray:
        while True:
            input(f"\n  Position the cup tip on corner [{name}] of the BLACK square.\n"
                  f"  Press [Enter] when in place... ")
            # let tf2 + spin run for a moment to be sure we have fresh data
            t_end = time.monotonic() + 0.3
            while time.monotonic() < t_end:
                rclpy.spin_once(self, timeout_sec=0.05)
            try:
                T = self.lookup_T_base_wrist()
            except RuntimeError as ex:
                print(f"  ERROR: {ex}")
                continue
            tip = self.compute_tip_in_base(T)
            w_xyz, w_rpy = matrix_to_xyz_rpy(T)
            print(f"  [{name}] wrist3 xyz=({w_xyz[0]:.4f}, {w_xyz[1]:.4f}, {w_xyz[2]:.4f})  "
                  f"rpy_deg=({np.degrees(w_rpy[0]):.1f}, {np.degrees(w_rpy[1]):.1f}, "
                  f"{np.degrees(w_rpy[2]):.1f})")
            print(f"  [{name}] tip    xyz=({tip[0]:.4f}, {tip[1]:.4f}, {tip[2]:.4f})")
            choice = input("  Accept this point? [Enter=yes, r=retry]: ").strip().lower()
            if choice != 'r':
                return tip

    def validate_touch_geometry(self, tl: np.ndarray, tr: np.ndarray,
                                bl: np.ndarray) -> list:
        """Return a list of failure-message strings (empty if all OK).

        A correct TL, TR, BL touch on a `size` x `size` square marker must satisfy:
          * |TL-TR| ≈ size              (side)
          * |TL-BL| ≈ size              (side)
          * |TR-BL| ≈ size * sqrt(2)    (diagonal, NOT zero — must not collapse)
          * angle (TR-TL) vs (BL-TL) ≈ 90 deg
        """
        size = self.marker_size
        side_diag = size * np.sqrt(2.0)
        side_tol_mm = max(5.0, 0.05 * size * 1000.0)  # 5 mm or 5% of side
        angle_tol_deg = 5.0

        d_tr = float(np.linalg.norm(tr - tl))
        d_bl = float(np.linalg.norm(bl - tl))
        d_diag = float(np.linalg.norm(tr - bl))

        errors = []
        if abs(d_tr - size) * 1000.0 > side_tol_mm:
            errors.append(f"|TL-TR| = {d_tr*1000:.1f} mm  but expected "
                          f"{size*1000:.1f} mm (±{side_tol_mm:.1f} mm).")
        if abs(d_bl - size) * 1000.0 > side_tol_mm:
            errors.append(f"|TL-BL| = {d_bl*1000:.1f} mm  but expected "
                          f"{size*1000:.1f} mm (±{side_tol_mm:.1f} mm).")
        if abs(d_diag - side_diag) * 1000.0 > side_tol_mm * 2:
            errors.append(f"|TR-BL| = {d_diag*1000:.1f} mm  but expected "
                          f"{side_diag*1000:.1f} mm (diagonal). "
                          "Most likely TR and BL are the SAME physical corner "
                          "— check which corner is which on the marker pattern.")

        # Angle between (TR-TL) and (BL-TL)
        v1 = (tr - tl)
        v2 = (bl - tl)
        n1 = np.linalg.norm(v1); n2 = np.linalg.norm(v2)
        if n1 > 1e-6 and n2 > 1e-6:
            cos_a = float(np.clip(np.dot(v1, v2) / (n1 * n2), -1.0, 1.0))
            angle_deg = float(np.degrees(np.arccos(cos_a)))
            if abs(angle_deg - 90.0) > angle_tol_deg:
                errors.append(f"angle(TL→TR, TL→BL) = {angle_deg:.1f} deg "
                              f"but expected 90 deg (±{angle_tol_deg:.1f} deg).")
        else:
            errors.append("TL→TR or TL→BL has zero length.")
        return errors

    def build_T_base_marker(self, tl: np.ndarray, tr: np.ndarray, bl: np.ndarray) -> np.ndarray:
        # Use the cv2.aruco / SOLVEPNP_IPPE_SQUARE canonical marker frame:
        #   X axis: TL → TR direction (top edge, left to right)
        #   Y axis: BL → TL direction (left edge, bottom to top)   <-- NOTE direction
        #   Z axis: X × Y  (out of the marker face, toward the camera viewer)
        # Match this convention here so the composition with the cv2-derived
        # T_camera_optical_marker is correct.
        x_raw = tr - tl
        y_raw = tl - bl                         # <-- IMPORTANT: TL - BL, not BL - TL
        x_len = float(np.linalg.norm(x_raw))
        y_len = float(np.linalg.norm(y_raw))
        diag = float(np.linalg.norm(tr - bl))
        print(f"\n  |TL-TR| = {x_len*1000:.2f} mm   |TL-BL| = {y_len*1000:.2f} mm   "
              f"|TR-BL| = {diag*1000:.2f} mm")
        print(f"  expected: side = {self.marker_size*1000:.2f} mm, "
              f"diag = {self.marker_size*np.sqrt(2)*1000:.2f} mm")
        if x_len < 1e-3 or y_len < 1e-3:
            raise RuntimeError("Touched points too close — re-run.")
        x_axis = x_raw / x_len
        # orthogonalize Y against X, then normalize
        y_axis = y_raw / y_len
        y_axis = y_axis - x_axis * float(np.dot(y_axis, x_axis))
        y_norm = float(np.linalg.norm(y_axis))
        if y_norm < 1e-4:
            raise RuntimeError(
                "After Gram-Schmidt, the Y axis collapsed to ~0. This means "
                "TL→TR and TL→BL are nearly parallel — typically because you "
                "touched the same physical corner for both TR and BL. Re-run "
                "the touch sequence after identifying the corners correctly."
            )
        y_axis /= y_norm
        z_axis = np.cross(x_axis, y_axis)
        z_axis /= float(np.linalg.norm(z_axis))

        R = np.column_stack([x_axis, y_axis, z_axis])
        # Marker center: from TL go half-size along +X (toward TR) and half-size
        # along -Y (toward BL, since +Y now goes BL→TL).
        center = tl + 0.5 * self.marker_size * x_axis - 0.5 * self.marker_size * y_axis
        return to_homog(R, center)

    # ──────────────────────────────────────────────────────── camera + detect
    def _info_cb(self, msg: CameraInfo):
        K = np.array(msg.k, dtype=np.float64).reshape(3, 3)
        D = np.array(msg.d, dtype=np.float64).reshape(-1) if len(msg.d) else np.zeros(5)
        with self.frame_lock:
            self.cam_K = K
            self.cam_D = D

    def _image_cb(self, msg: Image):
        try:
            frame = self.bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')
        except Exception as ex:
            self.get_logger().warn(f"cv_bridge failed: {ex}")
            return
        with self.frame_lock:
            self.latest_frame = frame

    def setup_camera_subs(self):
        if self.image_sub is not None:
            return
        # Use BEST_EFFORT on both subscribers: it is compatible with both
        # best-effort and reliable publishers (DDS asymmetric rule:
        # best-effort sub <- reliable pub  → OK,
        # reliable    sub <- best-effort pub → INCOMPATIBLE, no data flow).
        # Different realsense2_camera launches can publish camera_info with
        # either reliability, so best-effort sub is the safest.
        be_qos = QoSProfile(
            depth=10,
            reliability=QoSReliabilityPolicy.BEST_EFFORT,
            durability=QoSDurabilityPolicy.VOLATILE,
            history=QoSHistoryPolicy.KEEP_LAST,
        )
        self.image_sub = self.create_subscription(
            Image, self.image_topic, self._image_cb, be_qos
        )
        self.info_sub = self.create_subscription(
            CameraInfo, self.camera_info_topic, self._info_cb, be_qos
        )
        self.get_logger().info(
            f"Subscribed (best-effort QoS):\n"
            f"    image: '{self.image_topic}'\n"
            f"    info:  '{self.camera_info_topic}'"
        )

    def detect_marker_pose(self, frame, K, D):
        """Return (R, t) of the requested marker center in camera_color_optical_frame, or None."""
        gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
        if self.use_new_api:
            corners, ids, _ = self.aruco_detector.detectMarkers(gray)
        else:
            corners, ids, _ = cv2.aruco.detectMarkers(
                gray, self.aruco_dict, parameters=self.aruco_params
            )
        if ids is None or len(ids) == 0:
            return None
        idx = None
        for i, mid in enumerate(ids.flatten()):
            if int(mid) == self.marker_id:
                idx = i
                break
        if idx is None:
            return None
        s = self.marker_size / 2.0
        # cv2.aruco corner order: TL, TR, BR, BL (canonical marker orientation).
        # SOLVEPNP_IPPE_SQUARE REQUIRES this specific objPoints order:
        #   point 0 (TL): [-s, +s, 0]
        #   point 1 (TR): [+s, +s, 0]
        #   point 2 (BR): [+s, -s, 0]
        #   point 3 (BL): [-s, -s, 0]
        # With this convention: +X = TL→TR, +Y = BL→TL, +Z out of marker plane.
        objPoints = np.array([
            [-s, +s, 0.0],
            [+s, +s, 0.0],
            [+s, -s, 0.0],
            [-s, -s, 0.0],
        ], dtype=np.float32)
        imgPoints = corners[idx].reshape(-1, 2).astype(np.float32)
        try:
            ok, rvec, tvec = cv2.solvePnP(objPoints, imgPoints, K, D, flags=self.pnp_flag)
        except cv2.error:
            ok, rvec, tvec = cv2.solvePnP(
                objPoints, imgPoints, K, D, flags=cv2.SOLVEPNP_ITERATIVE
            )
        if not ok:
            return None
        R, _ = cv2.Rodrigues(rvec)
        return R, tvec.flatten()

    def save_debug_image(self, frame, K, D, out_path: Path):
        """Save a PNG with detected ArUco overlaid + per-marker pose axes drawn."""
        gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
        if self.use_new_api:
            corners, ids, _ = self.aruco_detector.detectMarkers(gray)
        else:
            corners, ids, _ = cv2.aruco.detectMarkers(
                gray, self.aruco_dict, parameters=self.aruco_params
            )
        vis = frame.copy()
        if ids is not None and len(ids) > 0:
            cv2.aruco.drawDetectedMarkers(vis, corners, ids)
            # Try to draw axes for each marker (helps you spot wrong scale).
            # Uses the IPPE_SQUARE canonical objPoints order.
            s = self.marker_size / 2.0
            objPoints = np.array([
                [-s, +s, 0.0], [+s, +s, 0.0],
                [+s, -s, 0.0], [-s, -s, 0.0],
            ], dtype=np.float32)
            for i in range(len(ids)):
                imgPoints = corners[i].reshape(-1, 2).astype(np.float32)
                try:
                    ok, rvec, tvec = cv2.solvePnP(
                        objPoints, imgPoints, K, D, flags=self.pnp_flag)
                except cv2.error:
                    ok, rvec, tvec = cv2.solvePnP(
                        objPoints, imgPoints, K, D, flags=cv2.SOLVEPNP_ITERATIVE)
                if ok:
                    try:
                        cv2.drawFrameAxes(vis, K, D, rvec, tvec, s * 1.5)
                    except Exception:
                        pass
                    px = int(np.mean(imgPoints[:, 0]))
                    py = int(np.mean(imgPoints[:, 1])) - 12
                    label = (f"id={int(ids[i])}  "
                             f"d={float(tvec[2]):.3f}m  "
                             f"size={self.marker_size*1000:.0f}mm")
                    cv2.putText(vis, label, (px, py),
                                cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 255, 255), 2,
                                cv2.LINE_AA)
        else:
            cv2.putText(vis, "NO MARKERS DETECTED", (20, 40),
                        cv2.FONT_HERSHEY_SIMPLEX, 1.0, (0, 0, 255), 2,
                        cv2.LINE_AA)
        cv2.imwrite(str(out_path), vis)

    def acquire_marker_samples(self):
        self.setup_camera_subs()

        timeout_sec = 15.0
        print(f"\n  Waiting up to {timeout_sec:.0f} s for camera_info AND the first image frame...")
        t0 = time.monotonic()
        last_log = 0.0
        while rclpy.ok():
            rclpy.spin_once(self, timeout_sec=0.1)
            with self.frame_lock:
                got_info = self.cam_K is not None
                got_img = self.latest_frame is not None
            if got_info and got_img:
                break
            # Periodic progress so the user sees what's missing.
            now = time.monotonic()
            if now - last_log > 2.0:
                last_log = now
                missing = []
                if not got_info: missing.append(f"camera_info '{self.camera_info_topic}'")
                if not got_img:  missing.append(f"image '{self.image_topic}'")
                print(f"    waiting on: {', '.join(missing)}  "
                      f"(elapsed {now - t0:.1f} s)")
            if now - t0 > timeout_sec:
                # Detailed failure report
                missing = []
                if not got_info: missing.append(f"camera_info '{self.camera_info_topic}'")
                if not got_img:  missing.append(f"image '{self.image_topic}'")
                raise RuntimeError(
                    "Timed out. Never received: " + " AND ".join(missing) +
                    f". Verify on a separate terminal:\n"
                    f"    ros2 topic hz {self.image_topic}\n"
                    f"    ros2 topic hz {self.camera_info_topic}\n"
                    f"  If 'hz' shows messages but this node times out, it's a QoS\n"
                    f"  mismatch — check `ros2 topic info -v <topic>` for the publisher QoS\n"
                    f"  and adjust accordingly. If 'hz' also hangs, the publisher itself\n"
                    f"  is not sending data on this topic (check the realsense2_camera\n"
                    f"  launch arguments)."
                )

        # Save a debug PNG with the detected marker(s) overlaid — user can
        # eyeball whether the right marker is being detected and at the right
        # depth before we trust 50 samples of it.
        with self.frame_lock:
            frame0 = self.latest_frame.copy()
            K0 = self.cam_K.copy()
            D0 = self.cam_D.copy()

        # ── Sanity check on the camera intrinsics ─────────────────────────
        img_h, img_w = frame0.shape[:2]
        fx, fy = float(K0[0, 0]), float(K0[1, 1])
        cx, cy = float(K0[0, 2]), float(K0[1, 2])
        print()
        print("  ── Camera intrinsics sanity check ──────────────────────────")
        print(f"    image size:  {img_w} x {img_h} px  (topic: {self.image_topic})")
        print(f"    K matrix:    fx={fx:.2f}  fy={fy:.2f}  cx={cx:.2f}  cy={cy:.2f}")
        print(f"    D (5 coef):  {np.array2string(D0[:5], precision=4)}")
        # Heuristic ratio: for a pinhole camera, the principal point should be
        # near image center (cx ≈ W/2, cy ≈ H/2) and the focal length in pixels
        # should be roughly comparable to the image width. The most common bug
        # is K being generated for a different resolution than the image.
        expected_fx_ratio = fx / max(img_w, 1)
        if expected_fx_ratio < 0.3 or expected_fx_ratio > 2.0:
            print(f"    WARNING: fx/width = {expected_fx_ratio:.2f} — looks unusual.")
            print(f"             For a normal lens fx ≈ 0.5–1.5 × image width.")
            print(f"             Likely cause: camera_info K was published for a")
            print(f"             DIFFERENT resolution than {img_w}x{img_h}.")
            print(f"             Check the realsense2_camera launch parameters.")
        if abs(cx - img_w / 2.0) > img_w * 0.1 or abs(cy - img_h / 2.0) > img_h * 0.1:
            print(f"    WARNING: principal point (cx, cy) is more than 10% off")
            print(f"             from image center ({img_w/2:.0f}, {img_h/2:.0f}).")
            print(f"             Possible camera_info / image resolution mismatch.")
        print()

        debug_dir = Path(self.output_dir) if self.output_dir else Path(__file__).resolve().parent
        debug_path = debug_dir / "calibration_debug_detection.png"
        self.save_debug_image(frame0, K0, D0, debug_path)
        print(f"  Debug detection image saved: {debug_path}")
        input("  Open it and check the marker overlay matches the printed marker.\n"
              "  Press [Enter] to start the 50-frame averaging... ")

        print(f"  Camera OK. Acquiring up to {self.num_samples} marker detections...")
        positions, quaternions = [], []
        attempts = 0
        max_attempts = self.num_samples * 15
        last_seen_id = None
        while len(positions) < self.num_samples and attempts < max_attempts:
            rclpy.spin_once(self, timeout_sec=0.05)
            attempts += 1
            with self.frame_lock:
                if self.latest_frame is None:
                    continue
                frame = self.latest_frame.copy()
                K = self.cam_K
                D = self.cam_D
            res = self.detect_marker_pose(frame, K, D)
            if res is None:
                continue
            R, t = res
            T = to_homog(R, t)
            q = quaternion_from_matrix(T)
            positions.append(t)
            quaternions.append(q)
            if len(positions) % 10 == 0 and len(positions) != last_seen_id:
                last_seen_id = len(positions)
                print(f"  collected {len(positions)} / {self.num_samples}")
        if len(positions) < 10:
            raise RuntimeError(
                f"Only {len(positions)} valid detections — check marker in FOV, "
                f"lighting, focus, and that marker_id={self.marker_id}."
            )
        print(f"  Done: {len(positions)} valid samples.")
        pos_mean, q_mean = average_poses(positions, quaternions)
        T_mean = quaternion_matrix(q_mean)
        T_mean[:3, 3] = pos_mean
        return T_mean, len(positions)

    # ──────────────────────────────────────────────────────────── tf2 helper
    def get_T_link_optical(self) -> np.ndarray:
        try:
            t = self.tf_buffer.lookup_transform(
                self.camera_link_frame, self.camera_optical_frame,
                Time(), timeout=Duration(seconds=2.0))
            T = transform_stamped_to_matrix(t)
            xyz, rpy = matrix_to_xyz_rpy(T)
            print(f"  T_{self.camera_link_frame}_{self.camera_optical_frame} (tf2): "
                  f"xyz=({xyz[0]:.4f}, {xyz[1]:.4f}, {xyz[2]:.4f}) "
                  f"rpy_deg=({np.degrees(rpy[0]):.3f}, {np.degrees(rpy[1]):.3f}, "
                  f"{np.degrees(rpy[2]):.3f})")
            return T
        except Exception as ex:
            self.get_logger().warn(
                f"tf2 {self.camera_link_frame} -> {self.camera_optical_frame} "
                f"lookup failed ({ex}); using RealSense standard rpy=(-pi/2, 0, -pi/2)."
            )
            return euler_matrix(-np.pi / 2, 0.0, -np.pi / 2)

    # ─────────────────────────────────────────────────────────────────── run
    def run(self):
        print("\n" + "=" * 72)
        print(" ArUco hand-eye calibration  —  fr3wml_industrial_bt")
        print("=" * 72)
        print(f"  marker_size    : {self.marker_size*1000:.1f} mm  (black square side)")
        print(f"  marker_id      : {self.marker_id}")
        print(f"  tool_z_offset  : {self.tool_z_offset*1000:.1f} mm  (wrist3 → cup tip)")
        print(f"  samples target : {self.num_samples}")
        print(f"  ArUco dict     : {self.aruco_dict_name}")
        print()
        print("  Make sure the robot is in DRAG-TEACH mode.")
        print("  The marker must be flat on the table and not move during calibration.")
        input("  Press [Enter] to start corner touch sequence... ")

        # Warm up tf2
        warm_end = time.monotonic() + 2.0
        while time.monotonic() < warm_end:
            rclpy.spin_once(self, timeout_sec=0.1)

        while True:
            tl = self.capture_corner("TL  (top-left)")
            tr = self.capture_corner("TR  (top-right)")
            bl = self.capture_corner("BL  (bottom-left)")

            errors = self.validate_touch_geometry(tl, tr, bl)
            if not errors:
                print("\n  Touch geometry sanity check: PASSED.")
                break

            print("\n  TOUCH GEOMETRY CHECK FAILED:")
            for e in errors:
                print(f"    - {e}")
            print("\n  Common cause: you touched TR and BL on the same physical")
            print("  corner. Make sure TL, TR, BL are 3 DISTINCT corners of the")
            print("  black square (look at the chev.me image to identify them in")
            print("  the marker's canonical orientation).")
            again = input("  Redo the 3 touches? [Enter=yes, q=abort]: ").strip().lower()
            if again == 'q':
                raise RuntimeError("Calibration aborted by user.")

        T_base_marker = self.build_T_base_marker(tl, tr, bl)
        xyz, rpy = matrix_to_xyz_rpy(T_base_marker)
        print("\n  T_base_marker_center:")
        print(f"     xyz = ({xyz[0]:.4f}, {xyz[1]:.4f}, {xyz[2]:.4f}) m")
        print(f"     rpy = ({np.degrees(rpy[0]):.2f}, {np.degrees(rpy[1]):.2f}, "
              f"{np.degrees(rpy[2]):.2f}) deg")

        print("\n  Move the robot OUT of the camera FOV (away from the marker).")
        print("  Marker must be fully visible and well-lit.")
        input("  Press [Enter] to start camera acquisition... ")

        T_optical_marker, n_samples = self.acquire_marker_samples()
        xyz_om, rpy_om = matrix_to_xyz_rpy(T_optical_marker)
        print("\n  T_camera_optical_marker_center (averaged over "
              f"{n_samples} frames):")
        print(f"     xyz = ({xyz_om[0]:.4f}, {xyz_om[1]:.4f}, {xyz_om[2]:.4f}) m")
        print(f"     rpy = ({np.degrees(rpy_om[0]):.2f}, {np.degrees(rpy_om[1]):.2f}, "
              f"{np.degrees(rpy_om[2]):.2f}) deg")

        # Compose final transforms
        T_base_optical = T_base_marker @ np.linalg.inv(T_optical_marker)
        T_link_optical = self.get_T_link_optical()
        T_base_link = T_base_optical @ np.linalg.inv(T_link_optical)

        xyz_opt,  rpy_opt  = matrix_to_xyz_rpy(T_base_optical)
        xyz_link, rpy_link = matrix_to_xyz_rpy(T_base_link)

        # ── Plausibility check on the result ───────────────────────────────
        # For a base-mounted RealSense looking at the workspace, the camera
        # is physically above base_link (z > 0.05 m) and somewhere ahead in X.
        suspicious = []
        if xyz_link[2] < 0.05:
            suspicious.append(
                f"camera_link Z={xyz_link[2]:.3f} m looks below the base. "
                f"Physical camera is mounted ABOVE base_link, so Z must be positive."
            )
        # Marker depth in camera optical frame: must be reasonable (10-150 cm).
        d_marker = float(np.linalg.norm(T_optical_marker[:3, 3]))
        if d_marker < 0.10 or d_marker > 1.50:
            suspicious.append(
                f"marker depth from camera = {d_marker*1000:.1f} mm — "
                f"physically you measured the marker ~30–50 cm from the camera. "
                f"This usually means camera_info K matrix is WRONG for the actual "
                f"image resolution (see the K sanity check above)."
            )
        if suspicious:
            print()
            print("  !! RESULT FAILS PLAUSIBILITY CHECK !!")
            for s in suspicious:
                print(f"     - {s}")
            print("     Do NOT paste this URDF until the root cause is fixed.")
            print()

        print("\n" + "=" * 72)
        print(" RESULT")
        print("=" * 72)
        print(f"  T_base_camera_optical:")
        print(f"     xyz = ({xyz_opt[0]:.4f}, {xyz_opt[1]:.4f}, {xyz_opt[2]:.4f}) m")
        print(f"     rpy = ({np.degrees(rpy_opt[0]):.3f}, {np.degrees(rpy_opt[1]):.3f}, "
              f"{np.degrees(rpy_opt[2]):.3f}) deg")
        print()
        print(f"  T_base_camera_link  (THIS goes into the URDF):")
        print(f"     xyz = ({xyz_link[0]:.6f}, {xyz_link[1]:.6f}, {xyz_link[2]:.6f}) m")
        print(f"     rpy = ({rpy_link[0]:.6f}, {rpy_link[1]:.6f}, {rpy_link[2]:.6f}) rad")
        print(f"     rpy = ({np.degrees(rpy_link[0]):.3f}, {np.degrees(rpy_link[1]):.3f}, "
              f"{np.degrees(rpy_link[2]):.3f}) deg")
        print()
        print("  URDF snippet — replace the camera_joint block in")
        print("  fr3wml_industrial_bt/urdf/fr_camera_gripper.urdf.xacro with:")
        print()
        print('    <joint name="camera_joint" type="fixed">')
        print('      <parent link="base_link"/>')
        print('      <child link="camera_link"/>')
        print(f'      <origin xyz="{xyz_link[0]:.6f} {xyz_link[1]:.6f} {xyz_link[2]:.6f}" '
              f'rpy="{rpy_link[0]:.6f} {rpy_link[1]:.6f} {rpy_link[2]:.6f}"/>')
        print('    </joint>')
        print()

        # Persist
        out_dir = self.output_dir if self.output_dir else str(Path(__file__).resolve().parent)
        ts = datetime.now().strftime('%Y%m%d_%H%M%S')
        out_path = Path(out_dir) / f"calibration_result_{ts}.yaml"
        result = {
            'timestamp': ts,
            'params': {
                'marker_size_m': self.marker_size,
                'marker_id': self.marker_id,
                'tool_z_offset_m': self.tool_z_offset,
                'num_samples_target': self.num_samples,
                'num_samples_used': int(n_samples),
                'aruco_dict': self.aruco_dict_name,
            },
            'corners_in_base_link_m': {
                'TL': [float(v) for v in tl],
                'TR': [float(v) for v in tr],
                'BL': [float(v) for v in bl],
            },
            'measured_side_lengths_m': {
                'TL_TR': float(np.linalg.norm(tr - tl)),
                'TL_BL': float(np.linalg.norm(bl - tl)),
            },
            'T_base_marker_center': {
                'xyz_m': [float(v) for v in xyz],
                'rpy_rad': [float(v) for v in rpy],
                'rpy_deg': [float(np.degrees(v)) for v in rpy],
            },
            'T_camera_optical_marker_center': {
                'xyz_m': [float(v) for v in xyz_om],
                'rpy_rad': [float(v) for v in rpy_om],
                'rpy_deg': [float(np.degrees(v)) for v in rpy_om],
            },
            'T_base_camera_optical': {
                'xyz_m': [float(v) for v in xyz_opt],
                'rpy_rad': [float(v) for v in rpy_opt],
                'rpy_deg': [float(np.degrees(v)) for v in rpy_opt],
            },
            'T_base_camera_link_URDF': {
                'xyz_m': [float(v) for v in xyz_link],
                'rpy_rad': [float(v) for v in rpy_link],
                'rpy_deg': [float(np.degrees(v)) for v in rpy_link],
            },
        }
        with open(out_path, 'w') as f:
            yaml.safe_dump(result, f, default_flow_style=False, sort_keys=False)
        print(f"  Saved YAML log: {out_path}")
        print()
        print("  Next steps:")
        print("    1. Open fr3wml_industrial_bt/urdf/fr_camera_gripper.urdf.xacro")
        print("    2. Replace the camera_joint origin with the xyz/rpy printed above")
        print("    3. cd ~/fr3wml_ws && colcon build --packages-select fr3wml_industrial_bt --symlink-install")
        print("    4. source install/setup.bash and rerun the vision launch")
        print("    5. In RViz with Fixed Frame=base_link, add MarkerArray on")
        print("       /detected_objects/axes_markers and verify the triad now sits")
        print("       precisely on the physical capsule.")
        print()


def main():
    rclpy.init()
    node = ArucoCalibrationNode()
    try:
        node.run()
    except KeyboardInterrupt:
        node.get_logger().info("Interrupted by user.")
    except Exception as ex:
        node.get_logger().fatal(f"Calibration failed: {ex}")
        import traceback
        traceback.print_exc()
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
