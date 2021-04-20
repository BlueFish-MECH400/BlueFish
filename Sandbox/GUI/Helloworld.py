import time
from datetime import datetime
import PyQt5.QtCore as qtc
import PyQt5.QtWidgets as qtw


class Helloworld(qtc.QThread):
    def __init__(self, helloworld_frequency: int):
        super(Helloworld, self).__init__(parent=None)
        self._start_time = time.perf_counter()
        self.helloworld_frequency = helloworld_frequency

        self.timer=qtc.QTimer()
        self.timer.timeout.connect(self.printing_helloworld)
        self.timer.moveToThread(self)

    def run(self):
        qtw.QApplication.sendPostedEvents()

        self.timer.start(self.helloworld_frequency)
        self.exec()
        

    def printing_helloworld(self):
        elapsed_time = time.perf_counter() - self._start_time
        # Output hello world to command window
        #print("Hello World!")
        print(elapsed_time)

    def stop(self):
        print('Stopping helloworld thread...', self.index)
        self.timer.stop()
        self.terminate()

