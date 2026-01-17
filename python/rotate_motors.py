#!/usr/bin/env python3
"""
Test script to rotate motors and read encoders
"""
import serial
import time

ser = serial.Serial('/dev/ttyACM0', 57600, timeout=1)
time.sleep(1)

print("Motor Rotation Test")
print("=" * 60)

try:
    # Test 1: Rotate at low speed (PWM 30)
    print("\n[1] Rotating motors at LOW speed (PWM 30, 30)...")
    ser.write(b'm 30 30\r')
    time.sleep(0.5)
    
    print("    Reading encoder data for 5 seconds...")
    start = time.time()
    while time.time() - start < 5:
        ser.write(b'f\r')
        time.sleep(0.1)
        if ser.in_waiting > 0:
            response = ser.readline().decode().strip()
            parts = response.split(',')
            if len(parts) == 8:
                print(f"    Enc: [{parts[0]:6}, {parts[1]:6}, {parts[2]:6}, {parts[3]:6}] | PWM: [{parts[4]:3}, {parts[5]:3}, {parts[6]:3}, {parts[7]:3}]")
    
    # Test 2: Rotate at medium speed
    print("\n[2] Rotating motors at MEDIUM speed (PWM 100, 100)...")
    ser.write(b'm 100 100\r')
    time.sleep(0.5)
    
    print("    Reading encoder data for 5 seconds...")
    start = time.time()
    while time.time() - start < 5:
        ser.write(b'f\r')
        time.sleep(0.1)
        if ser.in_waiting > 0:
            response = ser.readline().decode().strip()
            parts = response.split(',')
            if len(parts) == 8:
                print(f"    Enc: [{parts[0]:6}, {parts[1]:6}, {parts[2]:6}, {parts[3]:6}] | PWM: [{parts[4]:3}, {parts[5]:3}, {parts[6]:3}, {parts[7]:3}]")
    
    # Test 3: Rotate at high speed
    print("\n[3] Rotating motors at HIGH speed (PWM 200, 200)...")
    ser.write(b'm 200 200\r')
    time.sleep(0.5)
    
    print("    Reading encoder data for 5 seconds...")
    start = time.time()
    while time.time() - start < 5:
        ser.write(b'f\r')
        time.sleep(0.1)
        if ser.in_waiting > 0:
            response = ser.readline().decode().strip()
            parts = response.split(',')
            if len(parts) == 8:
                print(f"    Enc: [{parts[0]:6}, {parts[1]:6}, {parts[2]:6}, {parts[3]:6}] | PWM: [{parts[4]:3}, {parts[5]:3}, {parts[6]:3}, {parts[7]:3}]")
    
    # Stop motors
    print("\n[4] Stopping motors...")
    ser.write(b'm 0 0\r')
    time.sleep(0.5)
    
    print("\n✓ Test complete!")
    
except KeyboardInterrupt:
    print("\n\n[INTERRUPTED] Stopping motors...")
    ser.write(b'm 0 0\r')
    time.sleep(0.2)
    ser.close()
finally:
    ser.close()
