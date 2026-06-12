#!/usr/bin/env bash
# setup_groot2.sh
# ─────────────────────────────────────────────────────────────────────────────
# Groot2 setup for fr3wml_industrial_bt.
# Scarica Groot2 v1.9.0 (AppImage), crea lo shortcut desktop e verifica
# le dipendenze runtime (libzmq per il monitor live, FUSE per l'AppImage).
#
# Uso (una sola volta per macchina):
#   chmod +x src/fr3wml_industrial_bt/tools/setup_groot2.sh
#   src/fr3wml_industrial_bt/tools/setup_groot2.sh
# ─────────────────────────────────────────────────────────────────────────────
set -e

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
# workspace root: .../fr3wml_ws/src/fr3wml_industrial_bt/tools → ../../../
WS_ROOT="$(cd "${SCRIPT_DIR}/../../.." && pwd)"
GROOT2_DIR="${WS_ROOT}/groot2"                  # cartella di progetto Groot2
PKG_DIR="$(cd "${SCRIPT_DIR}/.." && pwd)"       # src/fr3wml_industrial_bt

GROOT2_VERSION="1.9.0"
GROOT2_FILE="Groot2-v${GROOT2_VERSION}-x86_64.AppImage"
GROOT2_URL="https://pub-32cef6782a9e411e82222dee82af193e.r2.dev/${GROOT2_FILE}"
GROOT2_PATH="${GROOT2_DIR}/${GROOT2_FILE}"

echo "════════════════════════════════════════════════════════"
echo "  Groot2 Setup — fr3wml_industrial_bt"
echo "  Version: ${GROOT2_VERSION}"
echo "  Install dir: ${GROOT2_DIR}"
echo "════════════════════════════════════════════════════════"
echo ""

mkdir -p "${GROOT2_DIR}"

# ── Step 1: Download if not already present ───────────────────────────────────
if [ -f "${GROOT2_PATH}" ]; then
    echo "✓ Groot2 AppImage già presente in: ${GROOT2_PATH}"
else
    echo "↓ Download di Groot2 v${GROOT2_VERSION}..."
    wget -q --show-progress "${GROOT2_URL}" -O "${GROOT2_PATH}"
    echo "✓ Scaricato: ${GROOT2_PATH}"
fi

# ── Step 2: Make executable ───────────────────────────────────────────────────
chmod +x "${GROOT2_PATH}"
echo "✓ Bit eseguibile impostato."

# ── Step 3: Optional desktop shortcut ─────────────────────────────────────────
DESKTOP_FILE="${HOME}/.local/share/applications/Groot2.desktop"
if [ ! -f "${DESKTOP_FILE}" ]; then
    mkdir -p "${HOME}/.local/share/applications"
    cat > "${DESKTOP_FILE}" <<EOF
[Desktop Entry]
Name=Groot2
Comment=BehaviorTree.CPP visual editor and monitor
Exec=${GROOT2_PATH} %U
Icon=utilities-system-monitor
Terminal=false
Type=Application
Categories=Development;Robotics;
StartupNotify=false
EOF
    echo "✓ Shortcut desktop creato in: ${DESKTOP_FILE}"
else
    echo "✓ Shortcut desktop già presente."
fi

# ── Step 4: Dependency check ──────────────────────────────────────────────────
echo ""
echo "── Controllo dipendenze runtime ─────────────────────────"

# ZMQ: serve per il monitor live (Groot2 ↔ bt_runner_node_visual)
if ldconfig -p 2>/dev/null | grep -q libzmq; then
    echo "✓ libzmq trovata  → monitoraggio BT live ABILITATO"
else
    echo "! libzmq NON trovata → installa con: sudo apt install libzmq3-dev"
    echo "  (l'editor statico funziona comunque senza ZMQ)"
fi

# FUSE: normalmente serve per avviare l'AppImage
if modinfo fuse &>/dev/null || [ -e /dev/fuse ]; then
    echo "✓ FUSE disponibile → l'AppImage può partire direttamente"
else
    echo "! FUSE non disponibile."
    echo "  Alternativa: estrai l'AppImage ed esegui il binario:"
    echo "    cd ${GROOT2_DIR}"
    echo "    ./${GROOT2_FILE} --appimage-extract"
    echo "    ./squashfs-root/usr/bin/groot2"
fi

echo ""
echo "════════════════════════════════════════════════════════"
echo "  Setup completato!"
echo ""
echo "  ▶ Avvia Groot2:"
echo "      ${GROOT2_PATH}"
echo ""
echo "  ▶ Apri il progetto Groot2 di fr3wml_industrial_bt:"
echo "      ${GROOT2_DIR}/fr3wml_industrial_bt.btproj"
echo ""
echo "  ▶ Modelli dei nodi (palette):"
echo "      ${GROOT2_DIR}/models/industrial_bt_framework_nodes.xml"
echo ""
echo "  ▶ Tree in editing visuale:"
echo "      ${PKG_DIR}/bt_trees/bottle_capsule_task_visual.xml"
echo ""
echo "  ▶ Monitor live (mentre gira industrial_bt_visual_demo.launch.py):"
echo "      Groot2 → Monitor → tcp://localhost:1666"
echo ""
echo "  ▶ Replay log post-esecuzione:"
echo "      cp /tmp/bt_trace.btlog ${GROOT2_DIR}/logs/"
echo "      Groot2 → Log Viewer → seleziona il file"
echo "════════════════════════════════════════════════════════"
