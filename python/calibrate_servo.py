#!/usr/bin/env python3
# -*- coding: utf-8 -*-

import os
import sys
import argparse
import time
from datetime import datetime
import yaml

# ---- Servo SDK Setup (wie in deinem Code) ----
CANDIDATE_PATHS = [
    "/home/rovi/ws_lidar/src/STServo_Python/stservo-env",
    "/home/rovi/ws_lidar/src/STServo_Python",
]
for p in CANDIDATE_PATHS:
    if os.path.isdir(p) and p not in sys.path:
        sys.path.insert(0, p)

try:
    from STservo_sdk import PortHandler, sts  # type: ignore
except Exception as e:
    print("❌ Konnte STservo_SDK nicht importieren. Prüfe Pfade/CANDIDATE_PATHS.\n", e)
    sys.exit(1)


def read_position_fallback(servo, servo_id: int) -> int:
    """
    Versucht verschiedene bekannte Methoden, um die aktuelle Positions-RAW-Zahl
    (0..4095/4096) auszulesen. Gibt int zurück oder wirft RuntimeError.
    """
    # 1) Häufig: ReadPosEx/ReadPos -> int
    for name in ("ReadPosEx", "ReadPos", "ReadPosition", "Read_Position", "read_position"):
        fn = getattr(servo, name, None)
        if callable(fn):
            try:
                pos = fn(servo_id)
                if isinstance(pos, (int, float)):
                    return int(round(pos))
            except Exception:
                pass

    # 2) Manche Libraries liefern ein Tupel/Liste (pos, speed, load, ...):
    for name in ("ReadPosEx", "ReadPos"):
        fn = getattr(servo, name, None)
        if callable(fn):
            try:
                result = fn(servo_id)
                if isinstance(result, (list, tuple)) and len(result) >= 1:
                    pos = result[0]
                    if isinstance(pos, (int, float)):
                        return int(round(pos))
            except Exception:
                pass

    raise RuntimeError(
        "Konnte Position nicht lesen. Prüfe SDK-Version – "
        "gibt es eine Funktion wie ReadPos/ReadPosEx/ReadPosition?"
    )


def write_enable_safe(servo, servo_id: int, enable: bool):
    """
    Schaltet den Servo-Treiber ein/aus. Manche SDKs nennen das WriteEnable,
    andere TorqueEnable/EnableTorque. Wir probieren Varianten.
    """
    candidates = [
        ("WriteEnable", (servo_id, 1 if enable else 0)),
        ("TorqueEnable", (servo_id, 1 if enable else 0)),
        ("EnableTorque", (servo_id, 1 if enable else 0)),
        ("Write_Enable", (servo_id, 1 if enable else 0)),
    ]
    last_err = None
    for name, args in candidates:
        fn = getattr(servo, name, None)
        if callable(fn):
            try:
                return fn(*args)
            except Exception as e:
                last_err = e
                continue
    if last_err:
        # nur warnen – manche Controller gehen auch ohne explizites Enable/Disable
        print(f"⚠️  Konnte {'Enable' if enable else 'Disable'} nicht setzen ({last_err}).")
    return None


def main():
    parser = argparse.ArgumentParser(
        description="Einmaliges Kalibrierungsskript: Mittellage per Hand einstellen und als mid_pos übernehmen."
    )
    parser.add_argument("--device", default="/dev/ttyACM1", help="Serielle Schnittstelle")
    parser.add_argument("--baud", type=int, default=1_000_000, help="Baudrate")
    parser.add_argument("--id", type=int, default=1, help="Servo-ID")
    parser.add_argument("--save-yaml", default="/home/rovi/ws_lidar/src/ros2-lidar-explorer/config/calibrate.yaml", help="Optionaler Pfad zu einer YAML-Datei für ROS2-Parameter")
    parser.add_argument("--node-name", default="servo_sweep_node", help="ROS2-Knotenname in der YAML")
    parser.add_argument("--ns", default="", help="ROS2-Namespace (optional), z.B. 'lidar' oder '/robot1/lidar'")
    parser.add_argument("--old-mid", type=int, default=2048, help="Bisher angenommener mid_pos (nur für Info-Ausgabe)")
    args = parser.parse_args()

    print("== STServo Mittel-Positions-Kalibrierung ==")
    print(f"Port: {args.device}  Baud: {args.baud}  ID: {args.id}")

    port = None
    servo = None
    try:
        port = PortHandler(args.device)
        servo = sts(port)

        if not port.openPort():
            raise RuntimeError(f"Port öffnen fehlgeschlagen: {args.device}")
        if not port.setBaudRate(args.baud):
            raise RuntimeError(f"Baudrate setzen fehlgeschlagen: {args.baud}")

        # (Optional) vorab Enable setzen, damit nachher Disable sicher klappt
        write_enable_safe(servo, args.id, True)
        time.sleep(0.05)

        # Strom AUS (entspannt), damit du manuell zentrieren kannst
        print("\n➡️  Servo wird stromlos/entspannt gemacht. Positioniere ihn jetzt exakt in die Mitte.")
        write_enable_safe(servo, args.id, False)
        print("… Bewege den Servo vorsichtig per Hand in die Mittellage.")
        input("Wenn du fertig bist: [Enter] drücken, um erneut zu aktivieren und Position zu lesen… ")

        # Strom AN
        write_enable_safe(servo, args.id, True)
        time.sleep(0.1)  # kurzes Stabilisieren

        # Position auslesen
        pos = read_position_fallback(servo, args.id)
        print(f"\n✅ Gelesene aktuelle Position (RAW): {pos}")

        # Winkel-Info gegenüber altem mid_pos (nur Info)
        raw_delta = pos - args.old_mid
        deg_offset = raw_delta * 360.0 / 4096.0  # 4096 Ticks ~ 360°
        print(f"Info: gegenüber altem mid_pos={args.old_mid} entspricht das einem Offset von {deg_offset:.3f}°.")

        # Empfehlung für deine Node:
        print("\n👉 Setze in deinem ROS2-Node den Parameter:")
        print(f"    mid_pos: {pos}")

        # Optional: YAML schreiben
        if args.save_yaml:
            # Namespace normalisieren
            ns_prefix = args.ns.strip()
            if ns_prefix and not ns_prefix.startswith("/"):
                ns_prefix = "/" + ns_prefix

            # Datenstruktur für ROS 2 Params
            if ns_prefix:
                data = {
                    ns_prefix: {
                        args.node_name: {
                            "ros__parameters": {
                                "mid_pos": int(pos),
                            }
                        }
                    }
                }
            else:
                data = {
                    args.node_name: {
                        "ros__parameters": {
                            "mid_pos": int(pos),
                        }
                    }
                }

            with open(args.save_yaml, "w", encoding="utf-8") as f:
                f.write(f"# Auto-generiert am {datetime.now().isoformat(timespec='seconds')}\n")
                yaml.safe_dump(data, f, sort_keys=False, allow_unicode=True)

            print(f"\n📝 YAML gespeichert: {args.save_yaml}")
            print("   (Mit ROS starten: --ros-args --params-file <pfad/zur/datei.yaml>)")

        print("\nFertig. Du kannst das Fenster schließen.")
    except KeyboardInterrupt:
        print("\nAbgebrochen.")
    except Exception as e:
        print(f"\n❌ Fehler: {e}")
        print("   Prüfe Verkabelung, ID, Baudrate und die exakte SDK-Funktionsbenennung.")
    finally:
        try:
            if servo:
                # Zum Schluss entspannt ausschalten (optional)
                write_enable_safe(servo, args.id, False)
        except Exception:
            pass
        if port:
            try:
                port.closePort()
            except Exception:
                pass


if __name__ == "__main__":
    main()
