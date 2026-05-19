import serial
import time

# Configure the serial port
# For Windows, use 'COM3', 'COM4', etc.
# For Linux/macOS, use '/dev/ttyUSB0' or '/dev/ttyACM0'
ser = serial.Serial(
    port='/dev/ttyAMA0',      # Replace with your specific port
    baudrate=200000,    # Standard communication speed
    timeout=1         # Read timeout in seconds
)

while True:
    if ser.is_open:
        print(f"Connected to {ser.port}")

    # 2. Wait for a short time for device processing
    time.sleep(0.1)

    # 3. Read response until a newline character
    if ser.in_waiting > 0: # Check if data is waiting in buffer
        response = ser.read(ser.in_waiting)

        # print out each byte in int with a space 
        for byte in response:
            print(byte, end=' ')
