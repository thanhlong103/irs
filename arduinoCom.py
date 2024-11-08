import serial
import time

# Set up the serial connection (adjust COM port and baud rate as necessary)
arduino = serial.Serial(port='COM6', baudrate=9600, timeout=1)
time.sleep(2)  # Allow time for the serial connection to initialize

def send_velocity(linear, angular):
    # Create the message in the format "<linear,angular>"
    message = f"<{linear},{angular}>"
    arduino.write(message.encode('utf-8'))  # Send the message

def read_xyz():
    while True:
        if arduino.in_waiting > 0:
            response = arduino.readline().decode('utf-8').strip()
            if response.startswith("<") and response.endswith(">"):
                # Remove the < and > characters and split by comma
                values = response[1:-1].split(',')
                if len(values) == 3:
                    x = float(values[0])
                    y = float(values[1])
                    z = float(values[2])
                    return x, y, z

while True:
    # Example usage: sending velocities and reading x, y, z
    send_velocity(0.1, 0.23)
    time.sleep(0.5)  # Small delay to wait for Arduino processing
    x, y, z = read_xyz()
    print(f"Received x: {x}, y: {y}, z: {z}")