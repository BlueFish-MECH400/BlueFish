# from PyQt5.uic import loadUi
#
# from PyQt5.QtWidgets import (QApplication, QTabWidget, QMainWindow, QLabel)
#
#
# import matplotlib as plt
# plt.use('Qt5Agg')
#
#
# class MainWindow(QMainWindow):
#     def __init__(self):
#         super(MainWindow, self).__init__()
#         loadUi("FishCommand.ui", self)
#         self
#
#
#
#
#
# if __name__ == '__main__':
#     app = QApplication([])
#     FishCommand = MainWindow()
#     FishCommand.show()
#     app.exec()


from FishCommander_Ui import Ui_FishCommand

from PyQt5 import QtWidgets as qtw
from PyQt5 import QtCore as qtc


class FishCommandWindow(qtw.QWidget, Ui_FishCommand):

    def __init__(self, *args, **kwargs):
        super().__init__(*args, **kwargs)
        self.setupUi(self)

        print(self.comboBox_operationMode.connect)

if __name__ == '__main__':
    app = qtw.QApplication([])
    widget = FishCommandWindow()
    widget.show()
    app.exec()
