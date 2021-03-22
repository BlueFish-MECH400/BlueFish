import os
import time
from datetime import datetime


class Logger:
    def __init__(self, fileName: str,
                 test_plan: str,
                 test_notes: str,
                 settings: dict):
        
        self.fileName = self.remove_bad_chars(fileName)
        self.test_plan = test_plan
        self.test_notes = test_notes
        self.settings = settings
        self.t0 = time.clock()
        
        # set folder and file path
        self.folderPath ='/home/pi/BlueFish/data/' + datetime.today().strftime("%Y-%m-%d") 
        self.filePath = self.folderPath + '/' + datetime.today().strftime('%Y-%m-%d - %H:%M:%S') + ' - ' + self.fileName + '.csv'

        # create folder (and file) as needed
        if not os.path.exists(self.folderPath):
            os.mkdir(self.folderPath)
        self.file = open(self.filePath, "a")
        print(self.filePath + " created")
        
        # Insert metadata
        standard_meta = {'start_timestamp': datetime.today().strftime('%Y-%m-%d - %H:%M:%S'),
                         'test_plan': self.test_plan,
                         'test_notes': self.test_notes or '',
                         'settings': self.settings,
                         }
        
        # insert meta into the top of the csv
        for meta in standard_meta:
            #append the data to the file
            self.file = open(self.filePath, "a")

            if meta == 'settings':
                self.file.write(' \n ####SETTINGS##### \n')
                for key in self.settings:
                    self.file.write(key + ',' + self.settings[key] + '\n')
            else:
                #write data with a new line
                self.file.write(meta + "," + standard_meta[meta] + '\n')
        
        # create headers
        self.file.write(' \n #######DATA######## \n')
        self.file.write('\n ELAPSED TIME [s], DEPTH [m], ALTITUDE [m], TEMP [C] \n') 
        self.file.close()

    def log_row(self, data: str):
        # append the data to the file
        self.file = open(self.filePath, "a")
        # write data with a new line
        self.file.write(str(time.clock() - self.t0)+ ',' + data + "\n")
        # close file
        self.file.close()

    @staticmethod
    def remove_bad_chars(string: str) -> str:
        for char in string:
            if char in r'\/:*?"<>|':
                string = string.replace(char, '')
        return string

