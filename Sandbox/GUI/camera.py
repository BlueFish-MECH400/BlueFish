import time
import subprocess
import os
from pathlib import Path
from datetime import datetime
import PyQt5.QtCore as qtc
import PyQt5.QtWidgets as qtw


class Camera(qtc.QThread):
    def __init__(self, photo_frequency: int):
        super(Camera, self).__init__(parent=None)
        self._start_time = time.perf_counter()
        self.photo_frequency = photo_frequency
        self.directory_name: str
        self.directory_path: str

        self.timer=qtc.QTimer()
        self.timer.timeout.connect(self.take_picture)

    def run(self):
        qtw.QApplication.sendPostedEvents()
        self.directory_name = datetime.today().strftime('%Y-%m-%d--%H:%M:%S')
        self.directory_path = "~/Pictures/" + self.directory_name
        os.chdir(Path.home())
        os.chdir("Pictures")
        os.makedirs(self.directory_name)

        self.timer.start(self.photo_frequency)
        exce()
        

    def take_picture(self):
        elapsed_time = time.perf_counter() - self._start_time
        photo_bash = "fswebcam -r 1920x1080 --no-banner " + self.directory_path + "/bluefish_" + str(elapsed_time) + ".jpg"
        subprocess.run(photo_bash, shell=True)

    def stop(self):
        print('Stopping camera thread...', self.index)
        self.timer.stop()
        self.terminate()


if __name__ == '__main__':
    camera = Camera(5000)
    camera.run()

