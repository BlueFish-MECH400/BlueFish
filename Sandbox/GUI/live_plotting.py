import sys
import time
import matplotlib
matplotlib.use('Qt5Agg')
from matplotlib.backends.backend_qt5agg import FigureCanvasQTAgg as FigureCanvas
from matplotlib.figure import Figure
import matplotlib.ticker as ticker
import queue
import numpy as np

from PyQt5 import QtCore as qtc
from PyQt5 import QtWidgets as qtw
from PyQt5 import QtWidgets, QtGui
from PyQt5 import uic
from PyQt5.QtCore import pyqtSlot


class MplCanvas(FigureCanvas):
	def __init__(self, parent=None, width=5, height=4, dpi=100):
		fig = Figure(figsize=(width, height), dpi=dpi)
		self.axes = fig.add_subplot(111)
		super(MplCanvas, self).__init__(fig)
		fig.tight_layout()


class Plotter(qtc.QThread):
	def __init__(self, index: int, settings: dict, filepath: str):
		super(Plotter, self).__init__(parent=None)
		self.filePath = filepath
		self.settings = settings
		self.index = index
		self.mutex = qtc.QMutex()


	def run(self):
		qtw.QApplication.sendPostedEvents()


	def stop(self):
		print('Stopping thread...', self.index)
		self.terminate()


