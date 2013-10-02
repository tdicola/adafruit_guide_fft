# SerialPortDevice.py
# Copyright 2013 Tony DiCola (tony@tonydicola.com)

# This is a concrete implementation of SpectrogramDevice to define communication
# with a serial port-based spectrogram device. 

import serial
import serial.tools.list_ports

import SpectrogramDevice


TIMEOUT_SECONDS = 5
PORT_BAUD_RATE = 38400


class SerialPortDevice(SpectrogramDevice.SpectrogramDevice):
	def __init__(self, path, name):
		"""Create a serial port device with a path to the serial port and name of the device.
		   Path and name should be values returned from calling enumerate_devices.
		"""
		self.path = path
		self.name = name

	def get_name(self):
		"""Return device name based on serial port name."""
		return self.name

	def get_fftsize(self):
		"""Return device FFT size."""
		return self.fftSize

	def get_samplerate(self):
		"""Return device sample rate in hertz."""
		return self.sampleRate

	def set_samplerate(self, samplerate):
		self.port.write('SET SAMPLE_RATE_HZ %d;' % samplerate)
		self.sampleRate = samplerate

	def get_magnitudes(self):
		"""Return an array of FFT magnitudes.  The number of values returned is the same as the FFT size."""
		self.port.write('GET MAGNITUDES;')
		return [float(self._readline()) for i in range(self.fftSize)]

	def open(self):
		"""Start communication with the device.  Must be done before any other calls are made to the device."""
		self.port = serial.Serial(self.path, PORT_BAUD_RATE, timeout=TIMEOUT_SECONDS, writeTimeout=TIMEOUT_SECONDS)
		# Read the initial state of the device
		self.port.write('GET FFT_SIZE;')
		self.fftSize = int(self._readline())
		self.port.write('GET SAMPLE_RATE_HZ;')
		self.sampleRate = int(self._readline())

	def close(self):
		"""Close communication with the device."""
		self.port.close()

	def _readline(self):
		value = self.port.readline()
		if value == None or value == '':
			raise IOError('Timeout exceeded while waiting for device to respond.')
		return value


def enumerate_devices():
	"""Enumerate all the serial ports."""
	return [SerialPortDevice(port[0], port[1]) for port in serial.tools.list_ports.comports() if port[2] != 'n/a']