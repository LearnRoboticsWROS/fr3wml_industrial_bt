// Fairino FR3WML CartesianBackend plugin for industrial_bt_framework.
//
// Wraps fairino_bridge's ExecutePoseMotion service (/fairino/movel_pose) so
// that generic BT primitives can command a linear TCP motion that is actually
// executed by the Fairino SDK `movel` under the hood.
//
// Incoming poses from the BT are TCP poses in the robot base frame. The
// Fairino SDK expects flange (wrist3) poses, so we apply a YAML-configurable
// flange offset (typically wrist3->flange translation) rotated by the target
// orientation before calling the service.

#include <array>
#include <chrono>
#include <cmath>
#include <memory>
#include <string>

#include <rclcpp/rclcpp.hpp>
#include <geometry_msgs/msg/pose.hpp>
#include <tf2/LinearMath/Quaternion.h>
#include <tf2/LinearMath/Matrix3x3.h>
#include <tf2/LinearMath/Vector3.h>
#include <pluginlib/class_list_macros.hpp>
#include <yaml-cpp/yaml.h>

#include "industrial_bt_framework/cartesian_backend.hpp"
#include "industrial_bt_framework/motion_profile.hpp"

#include "fairino_bridge/srv/execute_pose_motion.hpp"

namespace fr3wml_industrial_bt
{

class FairinoMoveLBackend : public industrial_bt_framework::CartesianBackend
{
public:
  FairinoMoveLBackend() = default;
  ~FairinoMoveLBackend() override = default;

  void initialize(
    rclcpp::Node::SharedPtr node,
    std::shared_ptr<moveit::planning_interface::MoveGroupInterface> /*move_group*/,
    const std::string & /*planning_group*/,
    const YAML::Node & params) override
  {
    node_ = node;

    service_name_          = "/fairino/movel_pose";
    flange_offset_         = {0.0, 0.0, 0.098};
    default_speed_percent_ = 50.0;
    tool_id_               = 0;
    user_id_               = 0;
    load_frames_before_motion_ = false;
    call_timeout_sec_      = 60.0;

    if (params && params.IsMap()) {
      if (params["cartesian_service_name"]) {
        service_name_ = params["cartesian_service_name"].as<std::string>();
      }
      if (params["flange_offset_xyz"] && params["flange_offset_xyz"].IsSequence()
          && params["flange_offset_xyz"].size() == 3) {
        flange_offset_[0] = params["flange_offset_xyz"][0].as<double>();
        flange_offset_[1] = params["flange_offset_xyz"][1].as<double>();
        flange_offset_[2] = params["flange_offset_xyz"][2].as<double>();
      }
      if (params["default_speed_percent"]) {
        default_speed_percent_ = params["default_speed_percent"].as<double>();
      }
      if (params["tool_id"]) tool_id_ = params["tool_id"].as<int>();
      if (params["user_id"]) user_id_ = params["user_id"].as<int>();
      if (params["load_frames_before_motion"]) {
        load_frames_before_motion_ = params["load_frames_before_motion"].as<bool>();
      }
      if (params["call_timeout_sec"]) {
        call_timeout_sec_ = params["call_timeout_sec"].as<double>();
      }
    }

    client_ = node_->create_client<fairino_bridge::srv::ExecutePoseMotion>(service_name_);

    RCLCPP_INFO(node_->get_logger(),
      "FairinoMoveLBackend initialised: service='%s', flange_offset=[%.3f, %.3f, %.3f], "
      "default_speed_percent=%.1f",
      service_name_.c_str(),
      flange_offset_[0], flange_offset_[1], flange_offset_[2],
      default_speed_percent_);
  }

  bool executeLinear(
    const geometry_msgs::msg::Pose & tcp_target_pose,
    const industrial_bt_framework::MotionProfile & profile,
    const std::string & description) override
  {
    if (!client_) {
      RCLCPP_ERROR(node_->get_logger(), "FairinoMoveLBackend: client not initialised");
      return false;
    }
    if (!client_->wait_for_service(std::chrono::seconds(5))) {
      RCLCPP_ERROR(node_->get_logger(),
        "FairinoMoveLBackend: service '%s' unavailable after 5 s",
        service_name_.c_str());
      return false;
    }

    const geometry_msgs::msg::Pose flange = wrist3ToFlange(tcp_target_pose);

    double speed = profile.movel_speed_percent;
    if (speed <= 0.0) speed = default_speed_percent_;

    auto req = std::make_shared<fairino_bridge::srv::ExecutePoseMotion::Request>();
    req->target_pose               = flange;
    req->motion_type               = "LINEAR";
    req->speed_percent             = static_cast<float>(speed);
    req->tool_id                   = tool_id_;
    req->user_id                   = user_id_;
    req->load_frames_before_motion = load_frames_before_motion_;

    RCLCPP_INFO(node_->get_logger(),
      "[%s] movel -> pos=(%.3f, %.3f, %.3f) speed=%.1f%%",
      description.c_str(),
      flange.position.x, flange.position.y, flange.position.z, speed);

    auto future = client_->async_send_request(req);
    auto status = future.wait_for(std::chrono::duration<double>(call_timeout_sec_));
    if (status != std::future_status::ready) {
      RCLCPP_ERROR(node_->get_logger(),
        "[%s] movel timed out after %.1f s", description.c_str(), call_timeout_sec_);
      return false;
    }

    auto resp = future.get();
    if (!resp->success) {
      RCLCPP_ERROR(node_->get_logger(),
        "[%s] movel failed: %s", description.c_str(), resp->message.c_str());
      return false;
    }
    return true;
  }

private:
  // Convert a wrist3_link pose to the corresponding flange pose.
  // The Fairino SDK operates in flange coordinates; the BT specifies
  // wrist3_link positions (matching the monolith convention).
  // flange = wrist3 + R * (0, 0, wrist3_to_flange_z)
  geometry_msgs::msg::Pose wrist3ToFlange(const geometry_msgs::msg::Pose & wrist3) const
  {
    tf2::Quaternion q(wrist3.orientation.x, wrist3.orientation.y,
                      wrist3.orientation.z, wrist3.orientation.w);
    tf2::Matrix3x3 R(q);
    tf2::Vector3 off(flange_offset_[0], flange_offset_[1], flange_offset_[2]);
    tf2::Vector3 off_base = R * off;

    geometry_msgs::msg::Pose flange = wrist3;
    flange.position.x += off_base.x();
    flange.position.y += off_base.y();
    flange.position.z += off_base.z();
    return flange;
  }

  rclcpp::Node::SharedPtr node_;
  rclcpp::Client<fairino_bridge::srv::ExecutePoseMotion>::SharedPtr client_;

  std::string          service_name_;
  std::array<double,3> flange_offset_;
  double               default_speed_percent_;
  int                  tool_id_;
  int                  user_id_;
  bool                 load_frames_before_motion_;
  double               call_timeout_sec_;
};

}  // namespace fr3wml_industrial_bt

PLUGINLIB_EXPORT_CLASS(
  fr3wml_industrial_bt::FairinoMoveLBackend,
  industrial_bt_framework::CartesianBackend)
