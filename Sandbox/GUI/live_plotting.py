import sys
import time
import matplotlib
matplotlib.use('Qt5Agg')
from matplotlib.backends.backend_qt5agg import FigureCanvasQTAgg as FigureCanvas
from matplotlib.figure import Figure
import matplotlib.ticker as ticker
import queue
import pandas as pd
from pandas import DataFrame as df

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
	def __init__(self, index: int, settings: dict, plot_settings: dict, filepath: str):
		super(Plotter, self).__init__(parent=None)
		self.filePath = filepath
		self.settings = settings
		self.plot_settings = plot_settings
		self.data = df()
		self.rows_to_plot = round(int(settings['Sample Rate']) * plot_settings['Elapsed Time [s]'])
		self.sleep_time = 2/(settings['Sample Rate']+1) if settings['Sample Rate'] == 100 \
			else 1/(settings['Sample Rate'] + 1)
		self.index = index
		self.mutex = qtc.QMutex()


	def run(self):
		qtw.QApplication.sendPostedEvents()

		while True:
			time.sleep(self.sleep_time)
			self.data = pd.read_csv(self.filename, header=22, usecols=[self.plot_settings['Y']])



	def stop(self):
		print('Stopping thread...', self.index)
		self.terminate()


