#!/usr/bin/env bash
set -euo pipefail

# --- Pfade/Parameter anpassen ---
BASE_DIR="/home/robi/ws_lidar/src/ros2-lidar-explorer"
VENV_ACTIVATE="$HOME/ws_lidar/src/STServo_Python/venv-servo/bin/activate"
PY_SCRIPT="$BASE_DIR/python/calibrate_servo.py"
YAML_DIR="$BASE_DIR/config"
YAML_OUT="$YAML_DIR/calibrate.yaml"

# Standard-Parameter (bei Bedarf hier ändern)
DEVICE="/dev/ttyACM0"
BAUD="1000000"
SERVO_ID="1"
NODE_NAME="servo_sweep_node"
NS=""              # optionaler Namespace, z.B. "lidar" (leer lassen = keiner)

echo "== Calibrate Servo Mid =="
echo "Arbeitsverzeichnis: $BASE_DIR"
echo "Python-Skript:      $PY_SCRIPT"
echo "YAML-Ziel:          $YAML_OUT"
echo

# --- Checks ---
if [[ ! -f "$PY_SCRIPT" ]]; then
  echo "❌ Python-Skript nicht gefunden: $PY_SCRIPT"
  exit 1
fi

if [[ ! -f "$VENV_ACTIVATE" ]]; then
  echo "❌ Virtuelle Umgebung nicht gefunden: $VENV_ACTIVATE"
  echo "Prüfe Pfad oder erstelle das venv erneut."
  exit 1
fi

# --- Venv aktivieren ---
# (liefert 'python' und Abhängigkeiten aus dem Servo-venv)
echo "👉 Aktiviere virtuelle Umgebung…"
# shellcheck disable=SC1090
source "$VENV_ACTIVATE"

# --- YAML-Ordner anlegen und ggf. alte Datei entfernen (erzwingt Überschreiben) ---
mkdir -p "$YAML_DIR"
if [[ -f "$YAML_OUT" ]]; then
  echo "🗑️  Entferne bestehende YAML: $YAML_OUT"
  rm -f "$YAML_OUT"
fi

# --- Startinfo ---
echo
echo "Starte Kalibrierung mit:"
echo "  Device:   $DEVICE"
echo "  Baud:     $BAUD"
echo "  Servo-ID: $SERVO_ID"
echo "  Node:     $NODE_NAME"
[[ -n "$NS" ]] && echo "  Namespace: $NS" || true
echo

# --- Skript ausführen ---
# Hinweis: Das Kalibrierungsskript macht den Servo stromlos, wartet auf [Enter],
# schaltet wieder ein und liest die aktuelle Position als mid_pos aus.
set -x
python "$PY_SCRIPT" \
  --device "$DEVICE" \
  --baud "$BAUD" \
  --id "$SERVO_ID" \
  --node-name "$NODE_NAME" \
  ${NS:+--ns "$NS"} \
  --save-yaml "$YAML_OUT"
set +x

echo
echo "✅ Fertig. YAML gespeichert unter: $YAML_OUT"
