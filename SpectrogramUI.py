# SpectrogramUI.py
# Copyright 2013 Tony DiCola (tony@tonydicola.com)

# User interface for Spectrogram program.

import matplotlib
matplotlib.use('Qt4Agg')
from matplotlib.animation import FuncAnimation
from matplotlib.backends.backend_qt4agg import FigureCanvasQTAgg as FigureCanvas
from matplotlib.cm import get_cmap
from matplotlib.figure import Figure
from matplotlib.gridspec import GridSpec
from matplotlib.ticker import MultipleLocator, FuncFormatter
import numpy as np
from PySide import QtCore, QtGui

VERSION = 'Spectrogram v1.0'

# Add global version number/name

class SpectrogramCanvas(FigureCanvas):
	def __init__(self, window):
		"""Initialize spectrogram canvas graphs."""
		# Initialize variables to default values.
		self.window = window
		self.samples = 100 # Number of samples to store
		self.fftSize = 256 # Initial FFT size just to render something in the charts
		self.sampleRate = 0
		self.binFreq = 0
		self.binCount = self.fftSize/2
		self.graphUpdateHz = 10 # Update rate of the animation
		self.coloredBin = None
		self.magnitudes = np.zeros((self.samples, self.binCount))
		# Tell numpy to ignore errors like taking the log of 0
		np.seterr(all='ignore')
		# Set up figure to hold plots
		self.figure = Figure(figsize=(1024,768), dpi=72, facecolor=(1,1,1), edgecolor=(0,0,0))
		# Set up 4x4 grid to hold 2 plots and colorbar
		gs = GridSpec(2, 2, height_ratios=[1,2], width_ratios=[9.5, 0.5])
		gs.update(left=0.075, right=0.925, bottom=0.05, top=0.95, wspace=0.05)
		# Set up frequency histogram bar plot
		self.histAx = self.figure.add_subplot(gs[0])
		self.histAx.set_title('Frequency Histogram')
		self.histAx.set_ylabel('Intensity (decibels)')
		self.histAx.set_xlabel('Frequency Bin (hz)')
		self.histAx.set_xticks([])
		self.histPlot = self.histAx.bar(np.arange(self.binCount), np.zeros(self.binCount), width=1.0, linewidth=0.0, facecolor='blue')
		# Set up spectrogram waterfall plot
		self.spectAx = self.figure.add_subplot(gs[2])
		self.spectAx.set_title('Spectrogram')
		self.spectAx.set_ylabel('Sample Age (seconds)')
		self.spectAx.set_xlabel('Frequency Bin (hz)')
		self.spectAx.set_xticks([])
		self.spectPlot = self.spectAx.imshow(self.magnitudes, aspect='auto', cmap=get_cmap('jet'))
		# Add formatter to translate position to age in seconds
		self.spectAx.yaxis.set_major_formatter(FuncFormatter(lambda x, pos: '%d' % (x*(1.0/self.graphUpdateHz))))
		# Set up spectrogram color bar
		cbAx = self.figure.add_subplot(gs[3])
		self.figure.colorbar(self.spectPlot, cax=cbAx, use_gridspec=True, format=FuncFormatter(lambda x, pos: '%d' % (x*100.0)))
		cbAx.set_ylabel('Intensity (decibels)')
		# Initialize canvas
		super(SpectrogramCanvas, self).__init__(self.figure)
		# Hook up mouse and animation events
		self.mpl_connect('motion_notify_event', self._mouseMove)
		self.ani = FuncAnimation(self.figure, self._update, interval=1000.0/self.graphUpdateHz, blit=False)

	def updateParameters(self, fftSize, sampleRate):
		"""Update the FFT size and sample rate parameters to redraw the charts appropriately."""
		# Update variables to new values.
		self.fftSize = fftSize
		self.sampleRate = sampleRate
		self.binCount = self.fftSize/2
		self.binFreq = self.sampleRate/float(self.fftSize)
		# Remove old bar plot.
		for bar in self.histPlot:
			bar.remove()
		# Update data for charts.
		self.histPlot = self.histAx.bar(np.arange(self.binCount), np.zeros(self.binCount), width=1.0, linewidth=0.0, facecolor='blue')
		self.magnitudes = np.zeros((self.samples, self.binCount))
		# Update frequency x axis to have 5 evenly spaced ticks from 0 to sampleRate/2.
		ticks = np.floor(np.linspace(0, self.binCount, 5))
		labels = ['%d hz' % i for i in np.linspace(0, self.sampleRate/2.0, 5)]
		self.histAx.set_xticks(ticks)
		self.histAx.set_xticklabels(labels)
		self.spectAx.set_xticks(ticks)
		self.spectAx.set_xticklabels(labels)

	def updateIntensityRange(self, low, high):
		"""Adjust low and high intensity limits for histogram and spectrum axes."""
		self.histAx.set_ylim(bottom=low, top=high)
		self.spectPlot.set_clim(low/100.0, high/100.0)

	def _mouseMove(self, event):
		# Update the selected frequency bin if the mouse is over a plot.
		# Check if sampleRate is not 0 so the status bar isn't updated if the spectrogram hasn't ever been started.
		if self.sampleRate != 0 and (event.inaxes == self.histAx or event.inaxes == self.spectAx):
			bin = int(event.xdata)
			self.window.updateStatus('Frequency bin %d: %.0f hz to %.0f hz' % (bin, bin*self.binFreq, (bin+1)*self.binFreq))
			# Highlight selected frequency in red
			if self.coloredBin != None:
				self.histPlot[self.coloredBin].set_facecolor('blue')
			self.histPlot[bin].set_facecolor('red')
			self.coloredBin = bin
		else:
			if self.coloredBin != None:
				self.histPlot[self.coloredBin].set_facecolor('blue')
			self.window.updateStatus()

	def _update(self, *fargs):
		# Animation function called 10 times a second to update graphs with recent data.
		# Get a list of recent magnitudes from the open device.
		mags = self.window.getMagnitudes()
		if mags != None:
			# Convert magnitudes to decibels.  Also skip the first value because it's
			# the average power of the signal, and only grab the first half of values
			# because the second half is for negative frequencies (which don't apply
			# to an FFT run on real data).
			mags = 20.0*np.log10(mags[1:len(mags)/2+1])
			# Update histogram bar heights based on magnitudes.
			for bin, mag in zip(self.histPlot, mags):
				bin.set_height(mag)
			# Roll samples forward and save the most recent sample.  Note that image
			# samples are scaled to 0 to 1.
			self.magnitudes = np.roll(self.magnitudes, 1, axis=0)
			self.magnitudes[0] = mags/100.0
			# Update spectrogram image data.
			self.spectPlot.set_array(self.magnitudes)
			return (self.histPlot, self.spectPlot)
		else:
			return ()


class MainWindow(QtGui.QMainWindow):
	def __init__(self, devices):
		"""Set up the main window.  
		   Devices should be a list of items that implement the SpectrogramDevice interface.
		"""
		super(MainWindow, self).__init__()
		self.devices = devices
		self.openDevice = None
		main = QtGui.QWidget()
		main.setLayout(self._setupMainLayout())
		self.setCentralWidget(main)
		self.status = self.statusBar()
		self.setGeometry(10,10,1024,768)
		self.setWindowTitle(VERSION)
		self._sliderChanged(0) # Force graphs to update their limits with initial values.
		self.show()

	def updateStatus(self, message=''):
		"""Update the status bar of the widnow with the provided message text."""
		self.status.showMessage(message)

	def getMagnitudes(self):
		"""Get a list of magnitudes if the device is open, or None if the device is not open.  
		   There are FFT size number of magnitudes.
		"""
		try:
			if self.isDeviceOpen():
				return self.openDevice.get_magnitudes()
		except IOError as e:
			self._communicationError(e)
		return None

	def isDeviceOpen(self):
		"""Return True if device is open."""
		return self.openDevice != None

	def _communicationError(self, error):
		# Error communicating with device, shut it down if possible.
		mb = QtGui.QMessageBox()
		mb.setText('Error communicating with device! %s' % error)
		mb.exec_()
		self._closeDevice()

	def _setupMainLayout(self):
		controls = QtGui.QVBoxLayout()
		controls.addWidget(QtGui.QLabel('<h3>%s</h3>' % VERSION))
		author = QtGui.QLabel('by <a href="http://www.github.com/tdicola/">Tony DiCola</a>')
		author.setOpenExternalLinks(True)
		controls.addWidget(author)
		controls.addSpacing(10)
		for control in self._setupControls():
			controls.addWidget(control)
		controls.addStretch(1)
		layout = QtGui.QHBoxLayout()
		layout.addLayout(controls)
		self.spectrogram = SpectrogramCanvas(self)
		layout.addWidget(self.spectrogram)
		return layout

	def _setupControls(self):
		# Set up device group		
		deviceCombo = QtGui.QComboBox()
		for device in sorted(self.devices, lambda a, b: cmp(a.get_name(), b.get_name())):
			deviceCombo.addItem(device.get_name(), userData=device)
		deviceBtn = QtGui.QPushButton('Open')
		deviceBtn.clicked.connect(self._deviceButton)
		self.deviceCombo = deviceCombo
		self.deviceBtn = deviceBtn
		device = QtGui.QGroupBox('Device')
		device.setLayout(QtGui.QGridLayout())
		device.layout().addWidget(QtGui.QLabel('Serial Port:'), 0, 0)
		device.layout().addWidget(deviceCombo, 0, 1)
		device.layout().addWidget(deviceBtn, 1, 1)
		# Set up device parameters group
		fftSize = QtGui.QLabel()
		sampleRate = QtGui.QLabel()
		modifyBtn = QtGui.QPushButton('Modify')
		modifyBtn.clicked.connect(self._modifyButton)
		self.fftSize = fftSize
		self.sampleRate = sampleRate
		self.modifyBtn = modifyBtn
		parameters = QtGui.QGroupBox('Device Parameters')
		parameters.setLayout(QtGui.QGridLayout())
		parameters.layout().addWidget(QtGui.QLabel('FFT Size:'), 0, 0)
		parameters.layout().addWidget(fftSize, 0, 1)
		parameters.layout().addWidget(QtGui.QLabel('Sample Rate:'), 1, 0)
		parameters.layout().addWidget(sampleRate, 1, 1)
		parameters.layout().addWidget(modifyBtn, 2, 1)
		parameters.setDisabled(True)
		self.parameters = parameters
		# Set up graph values group
		lowSlider = QtGui.QSlider(QtCore.Qt.Orientation.Horizontal)
		highSlider = QtGui.QSlider(QtCore.Qt.Orientation.Horizontal)
		lowSlider.setRange(0, 100)
		lowSlider.setValue(20)
		lowSlider.valueChanged.connect(self._sliderChanged)
		highSlider.setRange(0, 100)
		highSlider.setValue(60)
		highSlider.valueChanged.connect(self._sliderChanged)
		self.lowSlider = lowSlider
		self.highSlider = highSlider
		self.lowValue = QtGui.QLabel()
		self.highValue = QtGui.QLabel()
		graphs = QtGui.QGroupBox('Graphs')
		graphs.setLayout(QtGui.QGridLayout())
		graphs.layout().addWidget(QtGui.QLabel('Intensity Min:'), 0, 0)
		graphs.layout().addWidget(self.lowValue, 0, 1)
		graphs.layout().addWidget(lowSlider, 1, 0, 1, 2)
		graphs.layout().addWidget(QtGui.QLabel('Intensity Max:'), 2, 0)
		graphs.layout().addWidget(self.highValue, 2, 1)
		graphs.layout().addWidget(highSlider, 3, 0, 1, 2)
		return (device, parameters, graphs)

	def _deviceButton(self):
		# Toggle between opening and closing the device.
		if not self.isDeviceOpen():
			self._openDevice()
		else:
			self._closeDevice()

	def _modifyButton(self):
		# Create dialog
		dialog = QtGui.QDialog(self)
		dialog.setModal(True)
		sampleRate = QtGui.QSpinBox()
		sampleRate.setRange(1,9000)
		sampleRate.setValue(self.openDevice.get_samplerate())
		buttons = QtGui.QDialogButtonBox(QtGui.QDialogButtonBox.Ok | QtGui.QDialogButtonBox.Cancel)
		buttons.accepted.connect(dialog.accept)
		buttons.rejected.connect(dialog.reject)
		dialog.setLayout(QtGui.QVBoxLayout())
		dialog.layout().addWidget(QtGui.QLabel('Sample Rate (hz):'))
		dialog.layout().addWidget(sampleRate)
		dialog.layout().addWidget(buttons)
		dialog.setWindowTitle('Modify Device')
		# Show dialog and update device & UI based on results.
		try:
			if dialog.exec_() == QtGui.QDialog.Accepted:
				self.openDevice.set_samplerate(sampleRate.value())
				self._updateDeviceUI()
		except IOError as e:
			self._communicationError(e)

	def _updateDeviceUI(self):
		# Update UI to reflect current state of device.
		sampleRate = self.openDevice.get_samplerate()
		fftSize = self.openDevice.get_fftsize()
		self.fftSize.setText('%d' % fftSize)
		self.sampleRate.setText('%d hz' % sampleRate)
		self.spectrogram.updateParameters(fftSize, sampleRate)

	def _openDevice(self):
		try:
			# Open communication with selected device.
			device = self.deviceCombo.itemData(self.deviceCombo.currentIndex())
			self.updateStatus('Opening device %s...' % device.get_name())
			device.open()
			self.openDevice = device
			self.updateStatus('Communication with device %s established.' % device.get_name())
			# Update UI with data from open device.
			self._updateDeviceUI()
			self.parameters.setDisabled(False)
			self.deviceCombo.setDisabled(True)
			self.deviceBtn.setText('Close')
		except IOError as e:
			self.updateStatus() # Clear status bar
			self._communicationError(e)

	def _closeDevice(self):
		try:
			# Close the device if it's open.
			if self.isDeviceOpen():
				self.openDevice.close()
				self.openDevice = None
			# Update UI to show device is closed.
			self.parameters.setDisabled(True)
			self.deviceCombo.setDisabled(False)
			self.deviceBtn.setText('Open')
		except IOError as e:
			self._communicationError(e)

	def _sliderChanged(self, value):
		low = self.lowSlider.value()
		high = self.highSlider.value()
		# Adjust slider ranges to allowed values.
		self.lowSlider.setRange(0, high)
		self.highSlider.setRange(low, 100)
		# Update UI to represent new slider values
		self.lowValue.setText('%d dB' % low)
		self.highValue.setText('%d dB' % high)
		# Adjust chart UI with new slider values.
		self.spectrogram.updateIntensityRange(low, high)