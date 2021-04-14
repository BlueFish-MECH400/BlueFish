import time
from datetime import datetime
import PyQt5.QtCore as qtc
import PyQt5.QtWidgets as qtw


class Logger(qtc.QThread):
    def __init__(self, arduino, settings: dict, filepath: str):
        super(Logger, self).__init__(parent=None)
        self.ARDUINO = arduino
        self.filePath = filepath
        self.settings = settings
        self._start_time = time.perf_counter()
        self.sample_rate = settings['Sample Rate']

        if settings['Operation Mode'] != 0:
            self.file = open(self.filePath, "w")
            print(self.filePath + " created")
            self.insert_meta_and_headers()

    def run(self):
        qtw.QApplication.sendPostedEvents()
        while True:
            line = self.ARDUINO.readline().decode('utf-8').rstrip()
            elapsed_time = time.perf_counter() - self._start_time

            with self._lock:
                self.file = open(self.filePath, "a")
                self.file.write(f'{elapsed_time:0.4f} , {line} \n')
                self.file.close()

            time.sleep(1/self.sample_rate)

    def stop(self):
        print('Stopping thread...', self.index)
        self.terminate()

    def insert_meta_and_headers(self):
        # Insert metadata
        self.file.write('Start Time, ' + datetime.today().strftime('%Y-%m-%d - %H:%M:%S'))
        for key, value in self.settings.items():
            self.file.write(key + ',' + value + '\n')

        # create headers
        self.file.write(' \n #######DATA######## \n')
        self.file.write('\n Elapsed Time [s],Height [m],Height Error [m],Depth [m],Depth Error [m],Pressure [kPa],'
                        'Temperature [C],Yaw [deg],Pitch [deg], Battery Voltage [V],Battery Current [A]')
        self.file.close()

