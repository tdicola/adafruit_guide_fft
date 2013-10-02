# Spectrogram.py
# Copyright 2013 Tony DiCola (tony@tonydicola.com)

# Program to display a real-time spectrogram of data read from a device on the serial port.
# Depends on the following python libraries:
# - PySides (and Qt4)
# - Matplotlib
# - Numpy
# - PySerial
# For PySides, Matplotlib, and Numpy it is _highly_ recommended to install a prebuilt
# python distribution for scientific computing such as Anaconda or Enthought Canopy!

import sys

from PySide import QtGui

import SpectrogramUI
import SerialPortDevice


app = QtGui.QApplication(sys.argv)
devices = SerialPortDevice.enumerate_devices()
ui = SpectrogramUI.MainWindow(devices)
sys.exit(app.exec_())