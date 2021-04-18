import time
import subprocess
from datetime import datetime
import PyQt5.QtCore as qtc
import PyQt5.QtWidgets as qtw


class Camera(qtc.QThread):
    def __init__(self, photo_frequency: int):
        super(Camera, self).__init__(parent=None)
        self._start_time = time.perf_counter()
        self.photo_frequency = photo_frequency/1000

    def run(self):
        qtw.QApplication.sendPostedEvents()
        directory_name = datetime.today().strftime('%Y-%m-%d - %H:%M:%S')
        directory_bash = "mkdir " + directory_name
        subprocess.run(["cd Pictures/", directory_bash], shell=True)
        while True:
            elapsed_time = time.perf_counter() - self._start_time
            photo_bash = "fswebcam -r 1920x1080 --no-banner /home/pi/Pictures/" + directory_name + "/bluefish_" + str(elapsed_time) + ".jpg"
            subprocess.run(photo_bash, shell=true)

            time.sleep(self.photo_frequency)



    def stop(self):
        print('Stopping camera thread...', self.index)
        self.terminate()


if __name__ == '__main__':
    camera = Camera(1000)
    camera.run()

