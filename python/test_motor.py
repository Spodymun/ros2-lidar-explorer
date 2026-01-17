import serial
import time

ser = serial.Serial('/dev/ttyACM0', 57600, timeout=1)
time.sleep(1)

print("Testing 4-encoder Arduino firmware...")
print()

# Clear any buffered data
ser.reset_input_buffer()
ser.reset_output_buffer()
time.sleep(0.2)

print("[1] Testing Baudrate command...")
ser.write(b'b\r')
time.sleep(0.2)
if ser.in_waiting > 0:
    baud_response = ser.readline().decode().strip()
    print(f"    Response: {baud_response}")
else:
    print("    No response!")

# Clear buffers between commands
ser.reset_input_buffer()
ser.reset_output_buffer()
time.sleep(0.2)

print("[2] Testing Encoder Read command...")
ser.write(b'e\r')
time.sleep(0.2)
if ser.in_waiting > 0:
    encoder_response = ser.readline().decode().strip()
    print(f"    Response: {encoder_response}")
    encoders = encoder_response.split()
    print(f"    Parsed: {len(encoders)} values")
else:
    print("    No response!")

# Clear buffers between commands
ser.reset_input_buffer()
ser.reset_output_buffer()
time.sleep(0.2)

print("[3] Testing Reset Encoders command...")
ser.write(b'r\r')
time.sleep(0.2)
if ser.in_waiting > 0:
    reset_response = ser.readline().decode().strip()
    print(f"    Response: {reset_response}")
else:
    print("    No response!")

# Clear buffers between commands
ser.reset_input_buffer()
ser.reset_output_buffer()
time.sleep(0.2)

print("[4] Testing Motor Data command (f)...")
ser.write(b'f\r')
time.sleep(0.2)
if ser.in_waiting > 0:
    data_response = ser.readline().decode().strip()
    print(f"    Response: {data_response}")
else:
    print("    No response!")

print("\n✓ Test complete!")
ser.close()