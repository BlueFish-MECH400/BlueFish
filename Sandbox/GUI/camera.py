import time
import subprocess
import os
from datetime import datetime
import PyQt5.QtCore as qtc
import PyQt5.QtWidgets as qtw


class Camera(qtc.QThread):
    def __init__(self, photo_frequency: int):
        super(Camera, self).__init__(parent=None)
        self._start_time = time.perf_counter()
        self.photo_frequency = photo_frequency/1000
        self.timer=qtc.QTimer()

    def run(self):
        qtw.QApplication.sendPostedEvents()
        directory_name = datetime.today().strftime('%Y-%m-%d--%H:%M:%S')
        directory_path = "~/Pictures/" + directory_name
        os.mkdir(directory_path)

        
            


    def take_picture(self):
        elapsed_time = time.perf_counter() - self._start_time
        photo_bash = "fswebcam -r 1920x1080 --no-banner " + directory_path + "/bluefish_" + str(elapsed_time) + ".jpg"
        subprocess.run(photo_bash, shell=True)

        time.sleep(self.photo_frequency)

    def stop(self):
        print('Stopping camera thread...', self.index)
        self.terminate()


if __name__ == '__main__':
    camera = Camera(1000)
    camera.run()

