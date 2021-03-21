''' Send strings to an arduino from raspberry pi

Author: Nigel Swab

March 14, 2021
'''

import serial
import time

arduino = serial.Serial('/dev/ttyACM0', 9600, timeout=1)

# Get rid of garbage/incomplete data
arduino.flush()

#open file
f = open("params.txt", "r")
print(f.readline())
params = f.readline()

# Infinite loop
while (1):
    if params != f.readline():
        params = f.readline()

        # Send the string. Make sure you encode it before you send it to the Arduino.
        # arduino.write(send_string.encode('utf-8'))
        print(params)
        # Do nothing for 500 milliseconds (0.5 seconds)
        time.sleep(0.5)
    else:
    # Receive data from the Arduino
    #receive_string = arduino.readline().decode('utf-8').rstrip()

    # Print the data received from Arduino to the terminal
    #print(receive_string)
    print("nothing new")
    time.sleep(1)