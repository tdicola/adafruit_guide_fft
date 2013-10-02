# SpectrogramDevice.py
# Copyright 2013 Tony DiCola (tony@tonydicola.com)

# An abstract base class to implement for communication with a spectrogram device.
# This is meant for extending the spectrogram viewer UI so it can communicate with
# other devices in the future.


class SpectrogramDevice(object):
	def get_name(self):
		"""Return name of the device."""
		raise NotImplementedError()

	def get_fftsize(self):
		"""Return device's FFT size."""
		raise NotImplementedError()

	def set_fftsize(self, size):
		"""Set the device's FFT size to the specified number of bins."""
		raise NotImplementedError()

	def get_samplerate(self):
		"""Return device's sample rate in hertz."""
		raise NotImplementedError()

	def set_samplerate(self, frequency):
		"""Set the device's sample rate to the specified frequency in hertz."""
		raise NotImplementedError()

	def get_magnitudes(self):
		"""Return a list of magnitudes from an FFT run on the device.
		   The size of the returned magnitude list should be the sample
		   as the device's FFT size.
		"""
		raise NotImplementedError()

	def open(self):
		"""Open communication with the device."""
		raise NotImplementedError()

	def close(self):
		"""Close communication with the device."""
		raise NotImplementedError()