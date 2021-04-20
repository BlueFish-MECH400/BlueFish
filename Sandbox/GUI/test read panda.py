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

dframe = pd.DataFrame(columns=['lib', 'qty1', 'qty2'])
for i in range(5):
    dframe.loc[i] = ['name,name,name', 2, '3']

print(dframe)