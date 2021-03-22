# import serial
import time
import csv

import csv_logger

FILENAME = "Testing Logging"
TEST_PLAN = ''  # link to word test_plan on onedrive
TEST_NOTES = ''  # anything of note for this test
#
# dict for modes and their corresponding int run states on the Uno
MODE = {'STANDBY': '0', 'DEPTH': '1', 'ALTITUDE': '2'}

# # set usb port name, baud rate, and timeout so program doesn't get stuck
# ARDUINO = serial.Serial('/dev/ttyACM0', 9600, timeout=.1)
# # get rid of garbage/incomplete data
# ARDUINO.flush()


def main():
    settings = read_settings()
    update_settings(settings)
    log = csv_logger.Logger(FILENAME, TEST_PLAN, TEST_NOTES, settings)
    while 1:
        if check_settings(settings):
            settings = read_settings()
            log = csv_logger.Logger(FILENAME, TEST_PLAN, TEST_NOTES, settings)
        print("nah")
        time.sleep(1)


def setup_bluefish():
    log = csv_logger.Logger(fileName, test_plan, test_notes, settings)
    pass


def check_settings(old_settings: dict[str, str]) -> bool:
    ''' compare the current arduino settings to the settings in BlueFish_Settings.csv
        and update arduino if the BlueFish_Settings.csv has changed '''

    new_settings = read_settings()
    if old_settings != new_settings:
        update_settings(new_settings)
        return True
    else:
        return False


def read_settings() -> dict[str, str]:
    '''read BlueFish_Settings.csv and return dictionary with new arduino settings'''

    with open('BlueFish_Settings.csv') as settings_file:

        # setup csv_reader and initialize settings dict for BlueFish settings
        csv_reader = csv.reader(settings_file, delimiter=',')  # initialize csvfile object
        next(csv_reader, None)  # skip headers
        settings = {}  # initialize dictionary for BlueFish settings

        # populate BlueFish settings dict with csv settings
        for row in csv_reader:
            settings[row[0]] = row[1]

    return settings


def update_settings(new_settings: dict[str, str]) -> None:

    # generate interrupt

    # send each parameter into the arduino
    for key in new_settings:
        if key == 'MODE':
            send_string = (MODE[new_settings[key]] + ',')
        else:
            send_string = (new_settings[key] + ',')
        print(send_string)
        # ARDUINO.write(send_string.encode('utf-8'))

    return


main()