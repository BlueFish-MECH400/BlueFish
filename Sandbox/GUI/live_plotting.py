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

		# The window
		self.fig, self.ax = plt.subplots()
		x = range(-round(self.plot_settings['Elapsed Time [s]']), 0)
		line, = self.ax(x, self.data['Y1'])

		# Other
		self.index = index
		self.mutex = qtc.QMutex()

	def run(self):
		qtw.QApplication.sendPostedEvents()
		ax = plt.gca()
		self.data.plot(kind='line', y =[self.plot_settings['Y']], ax=ax)
		ax.set_xlabel("Elapsed Time [s]")
		plt.title('BlueFish Live Data')
		plt.show()
		# fig, ax = plt.subplot(1, 1)
		# ax.set_aspect('equal')
		ax.set_xlim(max(self.data['Elapsed Time [s]'] - float(self.plot_settings['Elapsed Time [s]']),
																		self.plot_settings['Elapsed Time [s]']))
		ax.set_ylim(-5, 5)
		# ax.hold(True)
		# x = self.data['Elapsed Time [s]']

		while True:
			time.sleep(self.sleep_time)
			self.data = pd.read_csv(self.filename, header=22, usecols=['Elapsed Time [s]', self.plot_settings['Y']])

	def animate(self):
		pass

	def stop(self):
		print('Stopping thread...', self.index)
		self.terminate()


