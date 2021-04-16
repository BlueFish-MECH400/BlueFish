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
import datetime

from PyQt5 import QtCore as qtc
from PyQt5 import QtWidgets as qtw
from PyQt5 import QtWidgets, QtGui
from PyQt5 import uic
from PyQt5.QtCore import pyqtSlot


data = pd.read_csv('2021_04_14 - 02.36.46 - trying with sensor API.csv',
                   header=22, usecols=[' Elapsed Time [s]', 'Height [m]', 'Temperature [C]'])
print(data)
