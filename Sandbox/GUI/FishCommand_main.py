from FishCommand import Ui_MainWindow

from PyQt5 import QtWidgets as qtw
from pandas import DataFrame as df

import csv

import time
from collections import OrderedDict as od
from datetime import datetime
from PyQt5 import QtCore as qtc
from PyQt5.QtWidgets import QFileDialog

# import serial
# import gpiozero

# INTERRUPT = gpiozero.LED(17)  # setup GPIO and ports for raspb interrupt pin 11 (GPIO 17)
# ARDUINO = serial.Serial('/dev/ttyACM0', 9600, timeout=.1)

class FishCommandWindow(qtw.QMainWindow, Ui_MainWindow):
    def __init__(self):
        super().__init__()
        # Setup GUI
        self.setupUi(self)
        self.connect_buttons()
        self.set_comboBox_data()
        self.show()

    def connect_buttons(self):
        '''connect signals from each button to their corresponding methods'''
        self.actionSave_Settings.triggered.connect(self.save_settings)
        self.actionLoad_Settings.triggered.connect(self.load_settings)
        self.pushButton_blueFishSettingsUpdate.clicked.connect(self.push_settings_to_bluefish)
        self.pushButton_updateLivePlotSettings.clicked.connect(self.update_plot_settings)
        self.pushButton_saveLivePlot.clicked.connect(self.save_plot)
        self.pushButton_photoSaveFolder.clicked.connect(self.choose_photo_directory)

    def set_comboBox_data(self):
        '''provide data values for combo boxes with units in text'''
        # sample rate in Hz
        self.comboBox_sampleRate.setItemData(0, 1)
        self.comboBox_sampleRate.setItemData(1, 5)
        self.comboBox_sampleRate.setItemData(2, 10)
        self.comboBox_sampleRate.setItemData(3, 25)
        self.comboBox_sampleRate.setItemData(4, 50)
        self.comboBox_sampleRate.setItemData(5, 100)
        # elapsed time on plot
        self.comboBox_plotTimeElapsed.setItemData(0, 5)  # 5 seconds
        self.comboBox_plotTimeElapsed.setItemData(0, 10)  # 10 seconds
        self.comboBox_plotTimeElapsed.setItemData(0, 30)  # 30 seconds
        self.comboBox_plotTimeElapsed.setItemData(0, 60)  # 1 minute
        self.comboBox_plotTimeElapsed.setItemData(0, 60*5)  # 5 minutes
        self.comboBox_plotTimeElapsed.setItemData(0, 60 * 10)  # 10 minutes
        self.comboBox_plotTimeElapsed.setItemData(0, 60 * 30)  # 30 minutes

    def save_settings(self):
        ''' Save csv with all metadata and settings '''
        option = qtw.QFileDialog.Options()
        file = qtw.QFileDialog.getSaveFileName(self, "Save BlueFish Settings", "Settings.csv", "*.csv", options=option)

        with open(file[0], "w", newline='\n') as f:
            user_settings = self.get_bluefish_settings()
            writer = csv.writer(f, delimiter=',')
            for label, data in user_settings.items():
                writer.writerow([label, data])

    def load_settings(self):
        '''allow user to choose csv file and load bluefish settings into GUI'''
        option = qtw.QFileDialog.Options()
        file = qtw.QFileDialog.getOpenFileName(self, "Load BlueFish Settings", "Settings.csv", "*.csv", options=option)
        with open(file[0], "r", newline='\n') as f:
            reader = csv.reader(f)
            settings = {rows[0]: rows[1] for rows in reader}
        self.set_bluefish_settings(settings)

    def get_bluefish_settings(self) -> od:
        '''create and return dictionary with user input settings'''
        user_settings = od([
            ('Sample Rate [Hz]', self.comboBox_sampleRate.currentIndex()),
            ('Operation Mode', self.comboBox_operationMode.currentIndex()),
            ('Target Depth [m]', self.doubleSpinBox_targetDepth.value()),
            ('Target Height [m]', self.doubleSpinBox_targetHeight.value()),
            ('Roll Kp', self.doubleSpinBox_rollP.value()),
            ('Roll Ki', self.doubleSpinBox_rollI.value()),
            ('Roll Kd', self.doubleSpinBox_rollD.value()),
            ('Height Kp', self.doubleSpinBox_heightP.value()),
            ('Height Ki', self.doubleSpinBox_heightI.value()),
            ('Height Kd', self.doubleSpinBox_heightD.value()),
            ('Depth Kp', self.doubleSpinBox_depthP.value()),
            ('Depth Ki', self.doubleSpinBox_depthI.value()),
            ('Depth Kd', self.doubleSpinBox_depthD.value()),
            ('Adaptive Depth Kp', self.doubleSpinBox_adaptiveP.value()),
            ('Adaptive Depth Ki', self.doubleSpinBox_adaptiveI.value()),
            ('Adaptive Depth Kd', self.doubleSpinBox_adaptiveD.value()),
            ('Camera Mode', self.comboBox_cameraMode.currentIndex()),
            ('Photo Frequency [ms]', self.spinBox_photoFrequency.value())])
        return user_settings

    def set_bluefish_settings(self, settings: dict) -> None:
        self.comboBox_sampleRate.setCurrentIndex(int(settings['Sample Rate [Hz]']))
        self.comboBox_operationMode.setCurrentIndex(int(settings['Operation Mode']))
        self.doubleSpinBox_targetDepth.setValue(float(settings['Target Depth [m]']))
        self.doubleSpinBox_targetHeight.setValue(float(settings['Target Height [m]']))
        self.doubleSpinBox_rollP.setValue(float(settings['Roll Kp']))
        self.doubleSpinBox_rollI.setValue(float(settings['Roll Ki']))
        self.doubleSpinBox_rollD.setValue(float(settings['Roll Kd']))
        self.doubleSpinBox_heightP.setValue(float(settings['Height Kp']))
        self.doubleSpinBox_heightI.setValue(float(settings['Height Ki']))
        self.doubleSpinBox_heightD.setValue(float(settings['Height Kd']))
        self.doubleSpinBox_depthP.setValue(float(settings['Depth Kp']))
        self.doubleSpinBox_depthI.setValue(float(settings['Depth Ki']))
        self.doubleSpinBox_depthD.setValue(float(settings['Depth Kd']))
        self.doubleSpinBox_adaptiveP.setValue(float(settings['Adaptive Depth Kp']))
        self.doubleSpinBox_adaptiveI.setValue(float(settings['Adaptive Depth Ki']))
        self.doubleSpinBox_adaptiveD.setValue(float(settings['Adaptive Depth Kd']))

    def push_settings_to_bluefish(self):
        pass
        # ''' get user input settings, interrupt arduino program to update arduino operational settings '''
        # settings = self.get_bluefish_settings()
        # INTERRUPT.on()
        # for setting, value in settings.items():
        #     if setting in ['Camera Mode', 'Photo Frequency [ms]',
        #                    'Adaptive Depth Kp', 'Adaptive Depth Ki', 'Adaptive Depth Kd']:
        #         pass
        #     else:
        #         send_string = (value[setting])
        #         print(send_string)  # for debug: comment me out
        #         ARDUINO.write(send_string.encode('utf-8'))
        # INTERRUPT.off()

    def choose_photo_directory(self):
        pass

    def get_plot_settings(self):
        pass

    def update_plot_settings(self):
        pass

    def save_plot(self):
        pass


if __name__ == '__main__':
    app = qtw.QApplication([])

    win = FishCommandWindow()
    app.exec_()
