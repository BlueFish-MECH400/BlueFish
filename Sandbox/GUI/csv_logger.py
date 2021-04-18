import time
from datetime import datetime
import PyQt5.QtCore as qtc
import PyQt5.QtWidgets as qtw


class Logger(qtc.QThread):
    def __init__(self, index: int, arduino, settings: dict, filepath: str):
        super(Logger, self).__init__(parent=None)
        self.ARDUINO = arduino
        self.filePath = filepath
        self.settings = settings
        self._start_time = time.perf_counter()
        self.sample_rate = settings['Sample Rate']
        self.index = index
        self.mutex = qtc.QMutex()

        if settings['Operation Mode'] != 'Standby':
            self.file = open(self.filePath, "a")
            print(self.filePath + " created")
            self.insert_meta_and_headers()
            self.file.close()

    def run(self):
        qtw.QApplication.sendPostedEvents()
        while True:
            line = self.ARDUINO.readline().decode('utf-8').rstrip()
            elapsed_time = time.perf_counter() - self._start_time

            # if not self.mutex.tryLock():
            #     print("Could not acquire mutex")
            #     return
    
            self.file = open(self.filePath, "a")
            self.file.write(str(elapsed_time) + ',' + line + '\n')
            self.file.close()
            # self.mutex.unlock()

            # time.sleep(1/(self.sample_rate+1))

    def stop(self):
        print('Stopping thread...', self.index)
        self.terminate()

    def insert_meta_and_headers(self):
        # Insert metadata
        self.file.write('Start Time, ' + datetime.today().strftime('%Y-%m-%d - %H:%M:%S') + '\n')
        for key, value in self.settings.items():
            self.file.write(key + ',' + str(value) + '\n')

        # create headers
        self.file.write(' \n #######DATA######## \n')
        self.file.write('\nElapsed Time [s],Height [m],Height Error [m],Depth [m],Depth Error [m],Pressure [kPa],'
                        'Temperature [C],Yaw [deg],Pitch [deg], Roll [deg], Battery Voltage [V],Battery Current [A]\n')
        self.file.close()

