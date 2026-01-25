#!/usr/bin/env python3

import serial
import sys
import time

def manual_calibrate(serial_port='/dev/ttyACM2'):
    """
    Manuelle Encoder-Kalibrierung
    
    Du drehst den Motor per Hand, das Script misst die Encoder-Ticks
    """
    
    try:
        ser = serial.Serial(serial_port, 57600, timeout=1)
        print(f"✓ Connected to {serial_port}\n")
        time.sleep(1)
    except Exception as e:
        print(f"✗ Error: {e}")
        return
    
    print("="*60)
    print("MANUELLE ENCODER-KALIBRIERUNG")
    print("="*60)
    print("\nAnleitung:")
    print("1. Markierung auf Reifen machen (z.B. mit Stift)")
    print("2. Markierung nach oben richten")
    print("3. ENTER drücken um zu STARTEN")
    print("4. Reifen MANUELL drehen (90° → 180° → 270° → 360°)")
    print("5. Nach jeder Viertels-Umdrehung ENTER drücken")
    print("6. Script zeigt Encoder-Ticks")
    print("="*60 + "\n")
    
    input("Drücke ENTER um zu starten...")
    
    # Initiale Messung
    print("\n📍 Lese initial Encoder-Werte...\n")
    
    def read_encoders():
        """Read and return encoder values"""
        ser.write(b'f\r')
        ser.flush()
        time.sleep(0.05)
        
        if ser.in_waiting > 0:
            line = ser.readline().decode('utf-8', errors='ignore').strip()
            try:
                parts = line.split(',')
                if len(parts) >= 4:
                    return int(parts[0]), int(parts[1]), int(parts[2]), int(parts[3])
            except:
                pass
        return None, None, None, None
    
    # Initial reading
    enc1_init, enc2_init, enc3_init, enc4_init = read_encoders()
    
    if enc1_init is None:
        print("✗ Keine Encoder-Daten!")
        ser.close()
        return
    
    print(f"INITIAL STATE:")
    print(f"  Motor 1: {enc1_init}")
    print(f"  Motor 2: {enc2_init}")
    print(f"  Motor 3: {enc3_init}")
    print(f"  Motor 4: {enc4_init}")
    print(f"  Left (1+2):  {(enc1_init + enc2_init) // 2}")
    print(f"  Right (3+4): {(enc3_init + enc4_init) // 2}")
    
    print("\n" + "="*60)
    print("Drehe jetzt den Reifen manuell LANGSAM!")
    print("Nach jeder Viertels-Umdrehung ENTER drücken")
    print("="*60 + "\n")
    
    marks = []
    
    for step in range(1, 5):
        input(f"Schritt {step}/4 (25% Rotation): Drücke ENTER wenn bereit...")
        
        enc1, enc2, enc3, enc4 = read_encoders()
        
        if enc1 is None:
            print("✗ Fehler beim Lesen")
            continue
        
        delta1 = enc1 - enc1_init
        delta2 = enc2 - enc2_init
        delta3 = enc3 - enc3_init
        delta4 = enc4 - enc4_init
        
        left_avg = (delta1 + delta2) // 2
        right_avg = (delta3 + delta4) // 2
        
        mark_data = {
            'step': step,
            'percent': step * 25,
            'enc1': delta1,
            'enc2': delta2,
            'enc3': delta3,
            'enc4': delta4,
            'left_avg': left_avg,
            'right_avg': right_avg
        }
        marks.append(mark_data)
        
        print(f"\n✓ MESSUNG SCHRITT {step}:")
        print(f"  Motor 1 Δ: {delta1:4d}")
        print(f"  Motor 2 Δ: {delta2:4d}")
        print(f"  Motor 3 Δ: {delta3:4d}")
        print(f"  Motor 4 Δ: {delta4:4d}")
        print(f"  Left Avg Δ:  {left_avg:4d} ticks")
        print(f"  Right Avg Δ: {right_avg:4d} ticks")
        print()
    
    print("\n" + "="*60)
    print("✓ KALIBRIERUNG ABGESCHLOSSEN!")
    print("="*60)
    
    final = marks[-1]
    
    print(f"\n📈 FINAL RESULTS (nach 360° Rotation):")
    print(f"  Motor 1: {final['enc1']:4d} ticks")
    print(f"  Motor 2: {final['enc2']:4d} ticks")
    print(f"  Motor 3: {final['enc3']:4d} ticks")
    print(f"  Motor 4: {final['enc4']:4d} ticks")
    print(f"  Left Avg (1+2):  {final['left_avg']:4d} ticks/rev")
    print(f"  Right Avg (3+4): {final['right_avg']:4d} ticks/rev")
    
    avg_ticks = (final['left_avg'] + final['right_avg']) // 2
    print(f"\n  🎯 AVERAGE: {avg_ticks} ticks per revolution")
    
    print(f"\n🔧 UPDATE in motor_publisher.py:")
    print(f"  self.TICKS_PER_REV = {avg_ticks}")
    
    print("\n" + "="*60)
    
    ser.close()

if __name__ == '__main__':
    port = sys.argv[1] if len(sys.argv) > 1 else '/dev/ttyACM0'
    manual_calibrate(port)
