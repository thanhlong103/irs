import keyboard  # Requires keyboard module for key detection
import serial
import time
import csv

# Open the serial connection to the Arduino
arduino = serial.Serial(port='/dev/ttyUSB1', baudrate=9600, timeout=.1)

# Open a CSV file for writing
with open('coordinates.csv', mode='w', newline='') as file:
    writer = csv.writer(file)
    writer.writerow(["Timestamp", "X", "Y"])  # Write header to CSV file

    def write_read(x):
        arduino.write(bytes(x, 'utf-8'))
        time.sleep(0.05)

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
                        print(x, y, z)
                        return x, y, z

    last_time = time.time()

    while True:
        # Check for key presses and send corresponding data to Arduino
        if keyboard.is_pressed('w'):
            write_read('1')
        elif keyboard.is_pressed('s'):
            write_read('0')
        elif keyboard.is_pressed('a'):
            write_read('2')
        elif keyboard.is_pressed('d'):
            write_read('3')
        elif keyboard.is_pressed('x'):
            write_read('4')
        elif keyboard.is_pressed('q'):
            write_read("5")
        elif keyboard.is_pressed('e'):
            write_read("6")

        # Read x, y, z from Arduino
        x, y, z = read_xyz()
        
        # Check if 10ms have passed since the last entry
        current_time = time.time()
        if current_time - last_time >= 0.5:  # 10ms
            timestamp = current_time
            writer.writerow([timestamp, x+(12), y+(-188)])  # Append data to CSV file
            last_time = current_time  # Update the last time
