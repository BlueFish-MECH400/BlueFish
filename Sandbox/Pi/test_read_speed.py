import csv
import time


def main():
    t0 = time.clock()
    for x in range(0, 10000):
        y = read_settings()
    e_time = time.clock() - t0
    print(e_time)


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


if __name__ == '__main__':
    main()