#!/usr/bin/env python3
import sys
import time
import threading
import serial

DEFAULT_PORT = "/dev/ttyACM0"
DEFAULT_BAUD = 57600

stop_flag = False


def reader_thread(ser: serial.Serial):
    """Continuously read lines from serial and print them with timestamps."""
    ser.reset_input_buffer()
    while not stop_flag:
        try:
            line = ser.readline().decode("utf-8", errors="replace").strip()
            if line:
                ts = time.strftime("%H:%M:%S")
                print(f"[RX {ts}] {line}")
        except Exception as e:
            print(f"[RX ERROR] {e}")
            time.sleep(0.2)


def poller_thread(ser: serial.Serial, interval_s: float = 1.0):
    """Send 'f' periodically to fetch encoder+output data."""
    while not stop_flag:
        try:
            cmd = "f\r"
            ser.write(cmd.encode("ascii", errors="ignore"))
            ser.flush()
            ts = time.strftime("%H:%M:%S")
            print(f"[TX {ts}] {cmd.strip()}  (poll)")
        except Exception as e:
            print(f"[POLL ERROR] {e}")
        time.sleep(interval_s)


def try_parse_f_line(line: str):
    """
    Attempt to parse the 'f' format:
    enc1,enc2,enc3,enc4,out1,out2,out3,out4
    """
    parts = line.split(",")
    if len(parts) < 8:
        return None
    try:
        enc = list(map(int, parts[:4]))
        out = list(map(int, parts[4:8]))
        return enc, out
    except ValueError:
        return None


def main():
    global stop_flag

    port = DEFAULT_PORT
    if len(sys.argv) > 1:
        port = sys.argv[1]

    baud = DEFAULT_BAUD
    if len(sys.argv) > 2:
        baud = int(sys.argv[2])

    print(f"Opening serial: port={port} baud={baud}")
    try:
        ser = serial.Serial(port, baud, timeout=1)
    except Exception as e:
        print(f"ERROR: Could not open serial port: {e}")
        sys.exit(1)

    # Start background RX reader
    t_rx = threading.Thread(target=reader_thread, args=(ser,), daemon=True)
    t_rx.start()

    # Start periodic poller (f)
    t_poll = threading.Thread(target=poller_thread, args=(ser, 1.0), daemon=True)
    t_poll.start()

    print("\n=== Arduino Debug Console ===")
    print("Commands you can type:")
    print("  m <L> <R>     -> legacy direct PWM (e.g. m 120 120)")
    print("  s <L> <R>     -> ticks/frame closed-loop (e.g. s 20 20)")
    print("  f             -> read motor data once")
    print("  e             -> read encoders once")
    print("  r             -> reset encoders + PID")
    print("  b             -> get baudrate")
    print("  raw: <text>   -> send exactly <text> (without the 'raw: ' prefix)")
    print("  q             -> quit")
    print("\nTip: After sending 's ...', watch the next 'f' lines: outputs should change and encoders should move.\n")

    try:
        while True:
            user = input("> ").strip()
            if not user:
                continue
            if user.lower() in ["q", "quit", "exit"]:
                break

            # allow raw mode
            if user.startswith("raw:"):
                payload = user[len("raw:"):].lstrip()
                cmd = payload + "\r"
                ser.write(cmd.encode("ascii", errors="ignore"))
                ser.flush()
                ts = time.strftime("%H:%M:%S")
                print(f"[TX {ts}] {payload}  (raw)")
                continue

            # normalize single-letter commands
            if user in ["f", "e", "r", "b"]:
                cmd = user + "\r"
                ser.write(cmd.encode("ascii", errors="ignore"))
                ser.flush()
                ts = time.strftime("%H:%M:%S")
                print(f"[TX {ts}] {user}")
                continue

            # m / s with args
            parts = user.split()
            if parts[0] in ["m", "s"] and len(parts) == 3:
                # validate ints
                try:
                    a1 = int(parts[1])
                    a2 = int(parts[2])
                except ValueError:
                    print("ERROR: arguments must be integers.")
                    continue

                cmd = f"{parts[0]} {a1} {a2}\r"
                ser.write(cmd.encode("ascii", errors="ignore"))
                ser.flush()
                ts = time.strftime("%H:%M:%S")
                print(f"[TX {ts}] {parts[0]} {a1} {a2}")

                # Immediately request an 'f' after sending to observe changes
                time.sleep(0.05)
                ser.write(b"f\r")
                ser.flush()
                print(f"[TX {time.strftime('%H:%M:%S')}] f  (after {parts[0]})")

                continue

            # fallback: send as typed
            cmd = user + "\r"
            ser.write(cmd.encode("ascii", errors="ignore"))
            ser.flush()
            ts = time.strftime("%H:%M:%S")
            print(f"[TX {ts}] {user}  (fallback)")

    except KeyboardInterrupt:
        pass
    finally:
        stop_flag = True
        time.sleep(0.2)
        try:
            ser.close()
        except Exception:
            pass
        print("Bye.")


if __name__ == "__main__":
    main()
