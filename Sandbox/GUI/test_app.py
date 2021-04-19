import os
import sys
import csv
from datetime import datetime

from camera import Camera
from csv_logger import Logger
from FishCommand import Ui_MainWindow

from PyQt5 import QtWidgets as qtw
from PyQt5 import QtCore as qtc

import serial
import gpiozero

INTERRUPT = gpiozero.LED(17)  # setup GPIO and ports for raspberry pi interrupt pin 11 (GPIO 17)

# Set the QtQuick Style
# Acceptable values: Default, Fusion, Imagine, Material, Universal.
os.environ['QT_QUICK_CONTROLS_STYLE'] = (sys.argv[1] if len(sys.argv) > 1 else "Default")


class FishCommandWindow(qtw.QMainWindow, Ui_MainWindow):
    def __init__(self):
        super().__init__()
        # Setup GUI
        self.setupUi(self)
        self.connect_buttons()
        self.set_combobox_data()
        self._is_logger_running = False
        self.logging_thread = qtc.QThread()
        self.plotting_thread = qtc.QThread()
        self.camera_thread = qtc.QThread()
        self.settings = {}
        self.displayed_settings = {}
        self.plot_settings = {}
        self.show()

    def connect_buttons(self):
        """Connect signals from each button to their corresponding methods"""

        self.actionSave_Settings.triggered.connect(self.save_settings)
        self.actionLoad_Settings.triggered.connect(self.load_settings)
        self.pushButton_blueFishSettingsUpdate.clicked.connect(self.push_settings_to_bluefish)
        self.pushButton_updateLivePlotSettings.clicked.connect(self.update_plot_settings)
        self.pushButton_saveLivePlot.clicked.connect(self.save_plot)
        self.pushButton_photoSaveFolder.clicked.connect(self.start_photographing)

    def set_combobox_data(self):
        """Provide data values for combo boxes with units in text"""

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
        """Save csv with all metadata and settings"""

        option = qtw.QFileDialog.Options()
        file = qtw.QFileDialog.getSaveFileName(self, "Save BlueFish Settings", "Settings.csv", "*.csv", options=option)
        if file[0]:
            with open(file[0], "w", newline='\n') as f:
                self.get_bluefish_settings()
                writer = csv.writer(f, delimiter=',')
                for label, data in self.settings.items():
                    writer.writerow([label, data])
        else:
            pass

    def load_settings(self):
        """Allow user to choose csv file and load bluefish settings into GUI"""

        option = qtw.QFileDialog.Options()
        file = qtw.QFileDialog.getOpenFileName(self, "Load BlueFish Settings", "Settings.csv", "*.csv", options=option)
        if file[0]:
            with open(file[0], "r", newline='\n') as f:
                reader = csv.reader(f)
                self.settings = {rows[0]: rows[1] for rows in reader}
            self.set_bluefish_settings()
        else:
            pass

    def get_bluefish_settings(self) -> None:
        """Update dictionary with user inputs"""

        self.settings = {
            'Sample Rate': self.comboBox_sampleRate.currentIndex(),
            'Operation Mode': self.comboBox_operationMode.currentIndex(),
            'Target Depth [m]': self.doubleSpinBox_targetDepth.value(),
            'Target Height [m]': self.doubleSpinBox_targetHeight.value(),
            'Roll Kp': self.doubleSpinBox_rollP.value(),
            'Roll Ki': self.doubleSpinBox_rollI.value(),
            'Roll Kd': self.doubleSpinBox_rollD.value(),
            'Height Kp': self.doubleSpinBox_heightP.value(),
            'Height Ki': self.doubleSpinBox_heightI.value(),
            'Height Kd': self.doubleSpinBox_heightD.value(),
            'Depth Kp': self.doubleSpinBox_depthP.value(),
            'Depth Ki': self.doubleSpinBox_depthI.value(),
            'Depth Kd': self.doubleSpinBox_depthD.value(),
            'Adaptive Depth Kp': self.doubleSpinBox_adaptiveP.value(),
            'Adaptive Depth Ki': self.doubleSpinBox_adaptiveI.value(),
            'Adaptive Depth Kd': self.doubleSpinBox_adaptiveD.value(),
            'Camera Mode': self.comboBox_cameraMode.currentIndex(),
            'Photo Frequency [ms]': self.spinBox_photoFrequency.value()}

    def set_bluefish_settings(self) -> None:
        """Set BlueCommand UI values to those from the saved settings"""

        self.comboBox_sampleRate.setCurrentIndex(int(self.settings['Sample Rate']))
        self.comboBox_operationMode.setCurrentIndex(int(self.settings['Operation Mode']))
        self.doubleSpinBox_targetDepth.setValue(float(self.settings['Target Depth [m]']))
        self.doubleSpinBox_targetHeight.setValue(float(self.settings['Target Height [m]']))
        self.doubleSpinBox_rollP.setValue(float(self.settings['Roll Kp']))
        self.doubleSpinBox_rollI.setValue(float(self.settings['Roll Ki']))
        self.doubleSpinBox_rollD.setValue(float(self.settings['Roll Kd']))
        self.doubleSpinBox_heightP.setValue(float(self.settings['Height Kp']))
        self.doubleSpinBox_heightI.setValue(float(self.settings['Height Ki']))
        self.doubleSpinBox_heightD.setValue(float(self.settings['Height Kd']))
        self.doubleSpinBox_depthP.setValue(float(self.settings['Depth Kp']))
        self.doubleSpinBox_depthI.setValue(float(self.settings['Depth Ki']))
        self.doubleSpinBox_depthD.setValue(float(self.settings['Depth Kd']))
        self.doubleSpinBox_adaptiveP.setValue(float(self.settings['Adaptive Depth Kp']))
        self.doubleSpinBox_adaptiveI.setValue(float(self.settings['Adaptive Depth Ki']))
        self.doubleSpinBox_adaptiveD.setValue(float(self.settings['Adaptive Depth Kd']))

    def push_settings_to_bluefish(self):
        """ get user input settings, interrupt arduino program to update arduino operational settings """

        if self._is_logger_running:
            self.stop_logging()
            self.stop_plotting()
        self.get_bluefish_settings()
        self.displayed_settings = self.settings
        self.displayed_settings['Operation Mode'] = self.comboBox_operationMode.currentText()
        self.displayed_settings['Sample Rate'] = self.comboBox_sampleRate.currentData()
        if self.settings['Operation Mode'] != 0:
            option = qtw.QFileDialog.Options()
            file = qtw.QFileDialog.getSaveFileName(self, "BlueFish Logging Data File",
                                                   (datetime.today().strftime('%Y_%m_%d - %H.%M') + ' - ' + '.csv'),
                                                   "*.csv", options=option)
            if file[0]:
                self.start_logging(file[0])
            else:
                self.comboBox_operationMode.setCurrentIndex(0)
                return

        # INTERRUPT.on()
        # for setting, value in settings.items():
        #     if setting in ['Camera Mode', 'Photo Frequency [ms]',
        #                    'Adaptive Depth Kp', 'Adaptive Depth Ki', 'Adaptive Depth Kd']:
        #         pass
        #     else:
        #         send_string = str(value)
        #         print(send_string)
        #         ARDUINO.write(send_string.encode('utf-8'))
        # INTERRUPT.off()

    def update_plot_settings(self):
        pass

    def start_logging(self, filepath):
        """Start a logging thread and connect all signals and slots"""

        self.logging_thread = Logger(0, ARDUINO, self.displayed_settings, filepath)
        self.logging_thread.start()
        self._is_logger_running = True

    def stop_logging(self):
        """Stop and eliminate logging thread, setting _is_logger_running to false"""

        self.logging_thread.stop()
        self._is_logger_running = False

    def get_plot_settings(self) -> None:
        """Update plot settings dictionary with current user input"""

        self.plot_settings = {
            'Y1': self.comboBox_plotY1.currentText(),
            'Y2': self.comboBox_plotY2.currentText(),
            'Y3': self.comboBox_plotY3.currentText()
        }

    def start_plotting(self, settings: dict, filepath):
        """Start a logging thread and connect all signals and slots"""

        self.plotting_thread = Plotter(1, settings, filepath)
        self.logging_thread.start()
        pass

    def stop_plotting(self):
        self.plotting_thread.stop()
        pass

    def save_plot(self):
        pass

    def choose_photo_directory(self):
        pass

    def start_photographing(self):
        self.camera_thread = Camera(5000)
        self.camera_thread.start()


if __name__ == '__main__':
    app = qtw.QApplication(sys.argv)

    win = FishCommandWindow()
    app.exec_()
