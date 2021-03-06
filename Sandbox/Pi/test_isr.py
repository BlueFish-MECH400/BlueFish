import gpiozero
import serial
import csv
import time

# dict for modes and their corresponding run states on the Uno
MODE = {'STANDBY': '0', 'DEPTH': '1', 'ALTITUDE': '2', 'SURFACE': '3'}

# setup GPIO and ports for raspberry pi
INTERRUPT = gpiozero.LED(17)  # interrupt pin 11 (GPIO 17)
ARDUINO = serial.Serial('/dev/ttyACM0', 9600, timeout=.1)

# Setup raspberry pi serial port and gpio
ARDUINO.flush()  # get rid of garbage/incomplete data
INTERRUPT.off()  # make sure interrupt is low


def main():
    while True:
        settings = read_settings()

        # create log and update arduino if settings mode isn't standby
        if settings['MODE'] != 'STANDBY':
            update_settings(settings)

        # while not in standby, log data and check BlueFish_Settings.csv for changes
        while settings['MODE'] != 'STANDBY':
            # Read until new line char and convert byte data into string, and log into csv
            line = ARDUINO.readline().decode('utf-8').rstrip()
            print(line)

            # check csv file for changes, update arduino if so and create new log
            new_settings = read_settings()
            if settings != new_settings:
                settings = update_settings(new_settings)


def read_settings() -> dict:
    ''' read BlueFish_Settings.csv and return dictionary with arduino settings '''

    with open('BlueFish_Settings.csv') as settings_file:

        # setup csv_reader and initialize settings dict for BlueFish settings
        csv_reader = csv.reader(settings_file, delimiter=',')  # initialize csvfile object
        settings = {}  # initialize dictionary for BlueFish settings

        # populate BlueFish settings dict with csv settings
        for row in csv_reader:
            settings[row[0]] = row[1]

    return settings


def update_settings(new_settings: dict) -> dict:
    ''' interupt arduino program to update arduino operational settings '''

    INTERRUPT.on()  # generate interrupt

    # send each parameter into the arduino
    for key in new_settings:
        if key in ['FILENAME', 'TEST_PLAN', 'TEST_PLAN', 'TEST_NOTES', '']:
            print('')
        elif key == 'MODE':
            send_string = (MODE[new_settings[key]] + ',')
            print(send_string)  # debug: comment me out
            ARDUINO.write(send_string.encode('utf-8'))
        else:
            send_string = (new_settings[key] + ',')
            print(send_string)  # for debug: comment me out
            ARDUINO.write(send_string.encode('utf-8'))

    INTERRUPT.off()

    return new_settings


if __name__ == '__main__':
    main()

