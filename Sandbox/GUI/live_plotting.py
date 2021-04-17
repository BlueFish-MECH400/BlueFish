import sys
import time
import matplotlib
matplotlib.use('Qt5Agg')
from matplotlib.backends.backend_qt5agg import FigureCanvasQTAgg as FigureCanvas
from matplotlib.figure import Figure
import matplotlib.pyplot as plt
import matplotlib.animation as animation
import matplotlib.ticker as ticker
import queue
import pandas as pd
from pandas import DataFrame as df

from PyQt5 import QtCore as qtc
from PyQt5 import QtWidgets as qtw
from PyQt5 import QtWidgets, QtGui
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
		# The filepath and settings
		self.filePath = filepath
		self.settings = settings
		self.plot_settings = plot_settings

		# empty dataframe for data with start of timer
		self.data = df()
		self.start_time = time.perf_counter()


		# The plot
		self.fig, self.ax = plt.subplots()
		self.num_rows = round(self.plot_settings['Elapsed Time [s]']) * self.settings['Sample Rate']
		# x_data = self.data[[-num_rows], ['Elapsed Time [s]']]
		self.y_data = pd.Series()
		self.x_data = pd.Series()


		# Other
		self.index = index
		self.mutex = qtc.QMutex()

	def run(self):
		qtw.QApplication.sendPostedEvents()
		ani = animation.FuncAnimation(self.fig, self.y_data, interval=10, blit=True, save_count=self.num_rows)
		plt.show()

		while True:
			time.sleep(self.sleep_time)
			self.data = pd.read_csv(self.filename, header=22, usecols=['Elapsed Time [s]', self.plot_settings['Y']])
			self.data = self.data.iloc[-self.num_rows:]
			self.x_data = self.data['Elapsed Time [s]']
			self.y_data = self.data[self.plot_settings['Y']]

	def animate(self):
		pass

	def stop(self):
		print('Stopping thread...', self.index)
		self.terminate()


