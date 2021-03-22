import serial
import time
import csv
import gpiozero
import schedule

import csv_logger

# dict for modes and their corresponding run states on the Uno
MODE = {'STANDBY': '0', 'DEPTH': '1', 'ALTITUDE': '2', 'SURFACE': '3'}

# setup GPIO and ports for raspberry pi
INTERRUPT = gpiozero.LED(17)  # interrupt pin 11 (GPIO 17)
ARDUINO = serial.Serial('/dev/ttyACM0', 9600, timeout=.1)
SETTINGS = {}

# TODO: CREATE CLASS FOR SETTINGS SO I CAN MORE EASILY USE A SCHEDULER? IDK, LOOK INTO SCHEDULING

def main():
    log, settings = setup_test()

    while True:
        # Read until new line char and convert byte data into string, and log into csv
        line = arduino.readline().decode('utf-8').rstrip()
        log.log_row(line)


def setup_test() -> list:
    ''' flush serial port, update arduino with settings, and create log '''

    # Setup raspberry pi
    ARDUINO.flush()  # get rid of garbage/incomplete data
    INTERRUPT.off()  # make sure interrupt is low

    # get settings, send them to arduino, and create initial log object
    global SETTINGS
    SETTINGS = read_settings()
    update_settings(SETTINGS)
    log = csv_logger.Logger(SETTINGS)

    return [log, settings]


def read_settings() -> dict:
    '''read BlueFish_Settings.csv and return dictionary with new arduino settings'''

    with open('BlueFish_Settings.csv') as settings_file:

        # setup csv_reader and initialize settings dict for BlueFish settings
        csv_reader = csv.reader(settings_file, delimiter=',')  # initialize csvfile object
        settings = {}  # initialize dictionary for BlueFish settings

        # populate BlueFish settings dict with csv settings
        for row in csv_reader:
            settings[row[0]] = row[1]

    return settings


def check_settings() -> None:
    # check if settings have changed. If so, update arduino and create new log
    settings = read_settings()
    if settings != SETTINGS:
        global SETTINGS
        SETTINGS = read_settings
        update_settings(SETTINGS)
        log = csv_logger.Logger(SETTINGS)


def update_settings(new_settings: dict) -> None:
    ''' interupt arduino program to update code settings'''

    INTERRUPT.on()  # generate interrupt

    # send each parameter into the arduino
    for key in new_settings:
        if key in ['FILENAME', 'TEST_PLAN', 'TEST_PLAN', 'TEST_NOTES', '']:
            print('')
        elif key == 'MODE':
            send_string = (MODE[new_settings[key]])
            print(send_string)  # debug: comment me out
            # ARDUINO.write(send_string.encode('utf-8'))
        else:
            send_string = (new_settings[key])
            print(send_string)  # debug: comment me out
            # ARDUINO.write(send_string.encode('utf-8'))

    INTERRUPT.off()
    return


main()