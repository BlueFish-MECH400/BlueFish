import csv

# dict for modes and their corresponding int run states on the Uno
MODE = {'STANDBY': '0', 'DEPTH': '1', 'ALTITUDE': '2'}


with open('BlueFish_Settings.csv') as settings_file:
    csv_reader = csv.reader(settings_file, delimiter=',')
    new_settings = {}
    for row in csv_reader:
        new_settings[row[0]] = row[1]

    # send each parameter into the arduino
    for key in new_settings:
        if key in ['FILENAME', 'TEST_PLAN', 'TEST_PLAN', 'TEST_NOTES', '']:
            print('')
        elif key == 'MODE':
            send_string = (MODE[new_settings[key]])
            print(send_string)
            # ARDUINO.write(send_string.encode('utf-8'))
        else:
            send_string = (new_settings[key])
            print(send_string)
            # ARDUINO.write(send_string.encode('utf-8'))