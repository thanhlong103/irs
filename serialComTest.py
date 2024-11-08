import keyboard  # Requires keyboard module for key detection
import serial
import time

arduino = serial.Serial(port='COM6',  baudrate=9600, timeout=.1)

def write_read(x):
    arduino.write(bytes(x,  'utf-8'))
    time.sleep(0.05)
    data = arduino.readline()
    return  data

while True:
    if keyboard.is_pressed('w'):
        data = write_read('1')
        print(data)
    elif keyboard.is_pressed('s'):
        data = write_read('0')
        print(data)
    elif keyboard.is_pressed('a'):
        data = write_read('2')
        print(data)
    elif keyboard.is_pressed('d'):
        data = write_read('3')
        print(data)
    elif keyboard.is_pressed('x'):
        data = write_read('4')
        print(data)
    elif keyboard.is_pressed('q'):
        data = write_read("5")
        print(data)
    elif keyboard.is_pressed('e'):
        data = write_read("6")
        print(data)
        
    # data = arduino.readline()
    # print(data)