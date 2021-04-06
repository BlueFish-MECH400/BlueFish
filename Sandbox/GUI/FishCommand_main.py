from FishCommand import Ui_MainWindow

from PyQt5.QtWidgets import QMainWindow as qmw
from PyQt5 import QtWidgets as qtw
from PyQt5 import QtCore as qtc


class FishCommand(qmw, Ui_MainWindow):
    def __init__(self, *args, **kwargs):
        super().__init__(*args, **kwargs)
        self.setupUi(self)


if __name__ == '__main__':
    app = qtw.QApplication([])
    widget = FishCommand()
    widget.show()
    app.exec()