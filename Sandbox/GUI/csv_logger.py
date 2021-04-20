import time
from datetime import datetime
import PyQt5.QtCore as qtc
import PyQt5.QtWidgets as qtw
from pandas import DataFrame as df


class Logger(qtc.QThread):
    def __init__(self, index: int, arduino, settings: dict, filepath: str):
        super(Logger, self).__init__(parent=None)
        self.ARDUINO = arduino
        self.filePath = filepath
        self.settings = settings
        self._start_time = time.perf_counter()
        self.sample_rate = settings['Sample Rate']
        self.data = df(columns=['Elapsed Time [s]', 'Height [m]', 'Height Error [m]' , 'Depth [m]' , 'Depth Error [m]',
                                'Pressure [kPa]', 'Temperature [C]', 'Yaw [deg]', 'Pitch [deg]', 'Roll [deg]',
                                'Battery Voltage [V]',' Battery Current [A]'])
        self.mutex = qtc.QMutex()

        if settings['Operation Mode'] != 0:
            self.file = open(self.filePath, "a")
            print(self.filePath + " created")
            self.insert_meta_and_headers()
            self.file.close()

    def run(self):
        qtw.QApplication.sendPostedEvents()
        index = 0
        while True:
            line = self.ARDUINO.readline().decode('utf-8').rstrip()
            elapsed_time = time.perf_counter() - self._start_time
            if line:
                parsed_data = line.split(',')
                self.data.iloc[index] = [elapsed_time, parsed_data[0], parsed_data[1], parsed_data[2], parsed_data[3],
                                         parsed_data[4], parsed_data[5], parsed_data[6], parsed_data[7], parsed_data[7],
                                         parsed_data[8], parsed_data[9], parsed_data[10], parsed_data[11], parsed_data[12]
                                         ]

                self.file = open(self.filePath, "a")
                self.file.write(str(elapsed_time) + ',' + line + '\n')
                self.file.close()

    def stop(self):
        print('Stopping logging thread')
        self.terminate()

    def insert_meta_and_headers(self):
        # Insert metadata
        self.file.write('Start Time, ' + datetime.today().strftime('%Y-%m-%d - %H:%M:%S') + '\n')
        for key, value in self.settings.items():
            self.file.write(key + ',' + str(value) + '\n')

        # create headers
        self.file.write(' \n #######DATA######## \n')
        self.file.write('\nElapsed Time [s],Height [m],Height Error [m],Depth [m],Depth Error [m],Pressure [kPa],'
                        'Temperature [C],Yaw [deg],Pitch [deg], Roll [deg], Battery Voltage [V],Battery Current [A] \n')
        self.file.close()

