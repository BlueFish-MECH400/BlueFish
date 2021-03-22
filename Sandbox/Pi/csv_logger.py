import os
import time
from datetime import datetime


class Logger:
    def __init__(self, settings: dict):
        
        self.fileName = self.remove_bad_chars(settings['FILENAME'])
        self.settings = settings
        self.t0 = time.clock()

        if self.settings['MODE'] != 'STANDBY':
            # set folder and file path
            self.folderPath ='/home/pi/BlueFish/data/' + datetime.today().strftime("%Y-%m-%d")
            self.filePath = self.folderPath + '/' + datetime.today().strftime('%Y-%m-%d - %H:%M:%S') + ' - ' + self.fileName + '.csv'

            # create folder (and file) as needed
            if not os.path.exists(self.folderPath):
                os.mkdir(self.folderPath)
            self.file = open(self.filePath, "a")
            print(self.filePath + " created")

            # Insert metadata
            self.file.write('Start Time, ' + datetime.today().strftime('%Y-%m-%d - %H:%M:%S'))
            for key in self.settings:
                self.file.write(key + ',' + self.settings[key] + '\n')

            # create headers
            self.file.write(' \n #######DATA######## \n')
            self.file.write('\n ELAPSED TIME [s], DEPTH [m], ALTITUDE [m], TEMP [C] \n')
            self.file.close()

    def log_row(self, data: str):
        # append the data to the file
        self.file = open(self.filePath, "a")
        # write data with a new line
        self.file.write(str(time.clock() - self.t0) + ',' + data + "\n")
        # close file
        self.file.close()

    @staticmethod
    def remove_bad_chars(string: str) -> str:
        for char in string:
            if char in r'\/:*?"<>|':
                string = string.replace(char, '')
        return string

