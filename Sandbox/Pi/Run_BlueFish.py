import serial
import time
import csv
import gpiozero

import csv_logger

# Metadata
FILENAME = ''  # suffix to add to csv filename
TEST_PLAN = ''  # link to word test_plan on onedrive
TEST_NOTES = ''  # anything of note for this test

# dict for modes and their corresponding int run states on the Uno
MODE = {'STANDBY': '0', 'DEPTH': '1', 'ALTITUDE': '2'}

# setup GPIO for interrupt
INTERRUPT = gpiozero.LED(17)

# set usb port name, baud rate, and timeout so program doesn't get stuck
ARDUINO = serial.Serial('/dev/ttyACM0', 9600, timeout=.1)
# get rid of garbage/incomplete data
ARDUINO.flush()


def main():
    settings = read_settings()
    update_settings(settings)
    log = csv_logger.Logger(FILENAME, TEST_PLAN, TEST_NOTES, settings)
    while 1:
        if check_settings(settings):
            settings = read_settings()
            log = csv_logger.Logger(FILENAME, TEST_PLAN, TEST_NOTES, settings)
        else:
            print("nah")
        time.sleep(1)


def check_settings(old_settings: dict) -> bool:
    ''' compare the current arduino settings to the settings in BlueFish_Settings.csv
        and update arduino if the BlueFish_Settings.csv has changed '''

    new_settings = read_settings()
    if old_settings != new_settings:
        update_settings(new_settings)
        return True
    else:
        return False


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


def update_settings(new_settings: dict) -> None:

    # generate interrupt
    INTERRUPT.on()

    # send each parameter into the arduino
    for key in new_settings:
        if key == 'MODE':
            send_string = (MODE[new_settings[key]] + ',')
        else:
            send_string = (new_settings[key] + ',')
        print(send_string)
        # ARDUINO.write(send_string.encode('utf-8'))

    INTERRUPT.off()
    return


main()