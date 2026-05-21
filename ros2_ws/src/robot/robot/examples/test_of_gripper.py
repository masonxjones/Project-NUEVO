import serial
import time

# --- CONFIGURATION ---
# Replace with your actual port (e.g., 'COM3' for Windows, '/dev/ttyACM0' for Linux)
SERIAL_PORT = '/dev/ttyS7'  
BAUD_RATE = 115200

def send_tlv_command(ser, msg_type, payload_bytes):
    """Packs and sends a Type-Length-Value packet over serial."""
    length = len(payload_bytes)
    # Construct packet: [Type] [Length] [Payload Data...] [Dummy CRC]
    packet = bytes([msg_type, length]) + bytes(payload_bytes) + b'\x00'
    
    print(f"Sending packet: {packet.hex().upper()}")
    ser.write(packet)

def main():
    print(f"Connecting to Arduino on {SERIAL_PORT}...")
    try:
        # Open serial port and wait for Arduino boot reset
        ser = serial.Serial(SERIAL_PORT, BAUD_RATE, timeout=0.1)
        time.sleep(2) 
        
        # --- DEFINING BURGER PIECE HEIGHTS (16-bit values split to 2 bytes) ---
        # Format: (High Byte, Low Byte)
        height_1_bottom_bun = [0x01, 0xF4]  # 500 steps
        height_2_patty      = [0x03, 0xE8]  # 1000 steps
        height_3_top_bun    = [0x05, 0xDC]  # 1500 steps
        
        # Array of sequences to loop through
        burger_sequence = [
            ("Bottom Bun", height_1_bottom_bun),
            ("Patty",      height_2_patty),
            ("Top Bun",    height_3_top_bun)
        ]
        
        for name, height_bytes in burger_sequence:
            input(f"\nPress Enter to execute drop and grab for: {name}...")
            
            # Send the MSG_BURGER_PICK (0x35) command
            send_tlv_command(ser, 0x35, height_bytes)
            
            # Read back any incoming [LOOP], [CTRL], or [UART] debug logs printed from Arduino
            print("Monitoring Arduino logs (4 seconds)...")
            start_time = time.time()
            while time.time() - start_time < 4.0:
                if ser.in_available() > 0:
                    line = ser.readline().decode('utf-8', errors='ignore').strip()
                    if line:
                        print(f"   [Arduino]: {line}")
                        
        print("\nBurger assembly sequence test complete.")
        ser.close()
        
    except Exception as e:
        print(f"Error: {e}")

if __name__ == "__main__":
    main()