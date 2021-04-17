"""
This program/class creates plots and updates them live as data is passed into it

Authors: Nigel Swab

Created: April 2021
"""
from PyQt5 import QTWidgets
from PyQt5.QtWidgets import QApplication, QMainWindow
import numpy as np
import matplotlib.pyplot as plt


class Plotter:
    plt.ion()  # Tell matplotlib you want interactive mode to plot live data

    def parse_vars(self, data: str) -> None:
        """ Take string of data and save/append them to lists"""
        pass

    def plot_data(self) -> None:
        """ Take new data and update plots/figures"""
        pass