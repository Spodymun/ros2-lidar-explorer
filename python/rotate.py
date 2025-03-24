import requests
import json
from urllib.parse import quote
import time
import sys
import math

# ESP IP-Adresse wird als Argument übergeben
ESP_IP = sys.argv[1]

# === Roboterparameter (aus deinem ESP-Node) ===
WHEEL_RADIUS = 0.035        # Meter
WHEEL_OFFSET_X = 0.197     # Radabstand
TICKS_PER_WHEEL_REV = 23    # Encoder-Ticks pro Radumdrehung

# === Geschwindigkeit der Drehung (L = -SPEED, R = +SPEED) ===
SPEED = 0.1 # Testweise, kann angepasst werden

# === Berechnung: Ticks für genau eine Roboterumdrehung ===
# Der Roboter dreht sich auf der Stelle → jedes Rad fährt einen Kreisbogen
# Eine Roboterumdrehung (360°) → jedes Rad fährt π * Spurweite
robot_rotation_path_per_wheel = math.pi * (2 * WHEEL_OFFSET_X)
wheel_circumference = 2 * math.pi * WHEEL_RADIUS
wheel_revolutions_for_1_robot_turn = robot_rotation_path_per_wheel / wheel_circumference
ticks_for_1_robot_turn = int(TICKS_PER_WHEEL_REV * wheel_revolutions_for_1_robot_turn)

print(f"→ Für eine Roboterdrehung: {wheel_revolutions_for_1_robot_turn:.2f} Radumdrehungen pro Rad")
print(f"→ Benötigte Encoder-Ticks: {ticks_for_1_robot_turn} Ticks pro Rad")

# === Funktionen ===

def fetch_encoder():
    """Liest aktuelle Encoderwerte vom ESP."""
    try:
        time.sleep(0.1)
        cmd = json.dumps({"T": 1001})
        url = f"http://{ESP_IP}/js?json={quote(cmd, safe='{}:,')}"
        r = requests.get(url, timeout=1)
        data = r.json()
        return data.get("odl", 0), data.get("odr", 0)
    except Exception as e:
        print(f"[!] Fehler beim Encoderlesen: {e}")
        return None, None

def send_motor_command(left, right):
    """Sendet Geschwindigkeit an ESP im geforderten Format."""
    try:
        cmd = json.dumps({"T": 1, "L": left, "R": right})
        url = f"http://{ESP_IP}/js?json={quote(cmd)}"
        requests.get(url, timeout=1)
    except Exception as e:
        print(f"[!] Fehler beim Senden: {e}")

def stop_robot():
    """Stoppt beide Motoren."""
    send_motor_command(0, 0)

def main():
    print("🔄 Starte 1x Drehung über Encoder...")

    # Start-Encoderwerte lesen
    start_l, start_r = fetch_encoder()
    if start_l is None or start_r is None:
        print("❌ Encoderwerte konnten nicht gelesen werden!")
        return

    print(f"📟 Startwerte: L={start_l}, R={start_r}")
    send_motor_command(-SPEED, SPEED)
    print("▶️ Drehen gestartet...")

    while True:
        send_motor_command(-SPEED, SPEED)
        time.sleep(0.2)
        _, current_r = fetch_encoder()
        if current_r is None:
            continue

        delta_ticks = abs(current_r - start_r)
        print(f"⏱️ Ticks: {delta_ticks}/{ticks_for_1_robot_turn}")

        if delta_ticks >= ticks_for_1_robot_turn:
            print("✅ Ziel erreicht: Genau eine Drehung.")
            break

    stop_robot()
    print("🛑 Roboter gestoppt.")

if __name__ == "__main__":
    main()
