// Audio Tone Input
// Copyright 2013 Tony DiCola (tony@tonydicola.com)

// This code is part of the guide at http://learn.adafruit.com/fft-fun-with-fourier-transforms/

#define ARM_MATH_CM4
#include <arm_math.h>
#include <Adafruit_NeoPixel.h>


////////////////////////////////////////////////////////////////////////////////
// CONIFIGURATION 
// These values can be changed to alter the behavior of the spectrum display.
////////////////////////////////////////////////////////////////////////////////

int SAMPLE_RATE_HZ = 9000;             // Sample rate of the audio in hertz.
const int TONE_LOWS[] = {              // Lower bound (in hz) of each tone in the input sequence.
  1723, 1934, 1512, 738, 1125
};
const int TONE_HIGHS[] = {             // Upper bound (in hz) of each tone in the input sequence.
  1758, 1969, 1546, 773, 1160
};
int TONE_ERROR_MARGIN_HZ = 50;         // Allowed fudge factor above and below the bounds for each tone input.
int TONE_WINDOW_MS = 4000;             // Maximum amount of milliseconds allowed to enter the full sequence.
float TONE_THRESHOLD_DB = 10.0;        // Threshold (in decibels) each tone must be above other frequencies to count.
const int FFT_SIZE = 256;              // Size of the FFT.  Realistically can only be at most 256 
                                       // without running out of memory for buffers and other state.
const int AUDIO_INPUT_PIN = 14;        // Input ADC pin for audio data.
const int ANALOG_READ_RESOLUTION = 10; // Bits of resolution for the ADC.
const int ANALOG_READ_AVERAGING = 16;  // Number of samples to average with each ADC reading.
const int POWER_LED_PIN = 13;          // Output pin for power LED (pin 13 to use Teensy 3.0's onboard LED).
const int NEO_PIXEL_PIN = 3;           // Output pin for neo pixels.
const int NEO_PIXEL_COUNT = 4;         // Number of neo pixels.  You should be able to increase this without
                                       // any other changes to the program.
const int MAX_CHARS = 65;              // Max size of the input command buffer


////////////////////////////////////////////////////////////////////////////////
// INTERNAL STATE
// These shouldn't be modified unless you know what you're doing.
////////////////////////////////////////////////////////////////////////////////

IntervalTimer samplingTimer;
float samples[FFT_SIZE*2];
float magnitudes[FFT_SIZE];
int sampleCounter = 0;
Adafruit_NeoPixel pixels = Adafruit_NeoPixel(NEO_PIXEL_COUNT, NEO_PIXEL_PIN, NEO_GRB + NEO_KHZ800);
char commandBuffer[MAX_CHARS];
int tonePosition = 0;
unsigned long toneStart = 0;


////////////////////////////////////////////////////////////////////////////////
// MAIN SKETCH FUNCTIONS
////////////////////////////////////////////////////////////////////////////////

void setup() {
  // Set up serial port.
  Serial.begin(38400);
  
  // Set up ADC and audio input.
  pinMode(AUDIO_INPUT_PIN, INPUT);
  analogReadResolution(ANALOG_READ_RESOLUTION);
  analogReadAveraging(ANALOG_READ_AVERAGING);
  
  // Turn on the power indicator LED.
  pinMode(POWER_LED_PIN, OUTPUT);
  digitalWrite(POWER_LED_PIN, HIGH);
  
  // Initialize neo pixel library and turn off the LEDs
  pixels.begin();
  pixels.show(); 
  
  // Clear the input command buffer
  memset(commandBuffer, 0, sizeof(commandBuffer));
  
  // Begin sampling audio
  samplingBegin();
}

void loop() {
  // Calculate FFT if a full sample is available.
  if (samplingIsDone()) {
    // Run FFT on sample data.
    arm_cfft_radix4_instance_f32 fft_inst;
    arm_cfft_radix4_init_f32(&fft_inst, FFT_SIZE, 0, 1);
    arm_cfft_radix4_f32(&fft_inst, samples);
    // Calculate magnitude of complex numbers output by the FFT.
    arm_cmplx_mag_f32(samples, magnitudes, FFT_SIZE);

    // Detect tone sequence.
    toneLoop();
  
    // Restart audio sampling.
    samplingBegin();
  }
    
  // Parse any pending commands.
  parserLoop();
}


////////////////////////////////////////////////////////////////////////////////
// UTILITY FUNCTIONS
////////////////////////////////////////////////////////////////////////////////

// Compute the average magnitude of a target frequency window vs. all other frequencies.
void windowMean(float* magnitudes, int lowBin, int highBin, float* windowMean, float* otherMean) {
    *windowMean = 0;
    *otherMean = 0;
    // Notice the first magnitude bin is skipped because it represents the
    // average power of the signal.
    for (int i = 1; i < FFT_SIZE/2; ++i) {
      if (i >= lowBin && i <= highBin) {
        *windowMean += magnitudes[i];
      }
      else {
        *otherMean += magnitudes[i];
      }
    }
    *windowMean /= (highBin - lowBin) + 1;
    *otherMean /= (FFT_SIZE / 2 - (highBin - lowBin));
}

// Convert a frequency to the appropriate FFT bin it will fall within.
int frequencyToBin(float frequency) {
  float binFrequency = float(SAMPLE_RATE_HZ) / float(FFT_SIZE);
  return int(frequency / binFrequency);
}

// Convert intensity to decibels
float intensityDb(float intensity) {
  return 20.0*log10(intensity);
}


////////////////////////////////////////////////////////////////////////////////
// SPECTRUM DISPLAY FUNCTIONS
///////////////////////////////////////////////////////////////////////////////

void toneLoop() {
  // Calculate the low and high frequency bins for the currently expected tone.
  int lowBin = frequencyToBin(TONE_LOWS[tonePosition] - TONE_ERROR_MARGIN_HZ);
  int highBin = frequencyToBin(TONE_HIGHS[tonePosition] + TONE_ERROR_MARGIN_HZ);
  // Get the average intensity of frequencies inside and outside the tone window.
  float window, other;
  windowMean(magnitudes, lowBin, highBin, &window, &other);
  window = intensityDb(window);
  other = intensityDb(other);
  // Check if tone intensity is above the threshold to detect a step in the sequence.
  if ((window - other) >= TONE_THRESHOLD_DB) {
    // Start timing the window if this is the first in the sequence.
    unsigned long time = millis();
    if (tonePosition == 0) {
      toneStart = time;
    }
    // Increment key position if still within the window of key input time.
    if (toneStart + TONE_WINDOW_MS > time) {
      tonePosition += 1;
    }
    else {
      // Outside the window of key input time, reset back to the beginning key.
      tonePosition = 0;
    }
  }
  // Check if the entire sequence was passed through.
  if (tonePosition >= sizeof(TONE_LOWS)/sizeof(int)) {
    toneDetected();
    tonePosition = 0;
  }
}

void toneDetected() {
  // Flash the LEDs four times.
  int pause = 250;
  for (int i = 0; i < 4; ++i) {
    for (int j = 0; j < NEO_PIXEL_COUNT; ++j) {
      pixels.setPixelColor(j, pixels.Color(255, 0, 0));
    }
    pixels.show();
    delay(pause);
    for (int j = 0; j < NEO_PIXEL_COUNT; ++j) {
      pixels.setPixelColor(j, 0);
    }
    pixels.show();
    delay(pause);
  }
}

////////////////////////////////////////////////////////////////////////////////
// SAMPLING FUNCTIONS
////////////////////////////////////////////////////////////////////////////////

void samplingCallback() {
  // Read from the ADC and store the sample data
  samples[sampleCounter] = (float32_t)analogRead(AUDIO_INPUT_PIN);
  // Complex FFT functions require a coefficient for the imaginary part of the input.
  // Since we only have real data, set this coefficient to zero.
  samples[sampleCounter+1] = 0.0;
  // Update sample buffer position and stop after the buffer is filled
  sampleCounter += 2;
  if (sampleCounter >= FFT_SIZE*2) {
    samplingTimer.end();
  }
}

void samplingBegin() {
  // Reset sample buffer position and start callback at necessary rate.
  sampleCounter = 0;
  samplingTimer.begin(samplingCallback, 1000000/SAMPLE_RATE_HZ);
}

boolean samplingIsDone() {
  return sampleCounter >= FFT_SIZE*2;
}


////////////////////////////////////////////////////////////////////////////////
// COMMAND PARSING FUNCTIONS
// These functions allow parsing simple commands input on the serial port.
// Commands allow reading and writing variables that control the device.
//
// All commands must end with a semicolon character.
// 
// Example commands are:
// GET SAMPLE_RATE_HZ;
// - Get the sample rate of the device.
// SET SAMPLE_RATE_HZ 400;
// - Set the sample rate of the device to 400 hertz.
// 
////////////////////////////////////////////////////////////////////////////////

void parserLoop() {
  // Process any incoming characters from the serial port
  while (Serial.available() > 0) {
    char c = Serial.read();
    // Add any characters that aren't the end of a command (semicolon) to the input buffer.
    if (c != ';') {
      c = toupper(c);
      strncat(commandBuffer, &c, 1);
    }
    else
    {
      // Parse the command because an end of command token was encountered.
      parseCommand(commandBuffer);
      // Clear the input buffer
      memset(commandBuffer, 0, sizeof(commandBuffer));
    }
  }
}

// Macro used in parseCommand function to simplify parsing get and set commands for a variable
#define GET_AND_SET(variableName) \
  else if (strcmp(command, "GET " #variableName) == 0) { \
    Serial.println(variableName); \
  } \
  else if (strstr(command, "SET " #variableName " ") != NULL) { \
    variableName = (typeof(variableName)) atof(command+(sizeof("SET " #variableName " ")-1)); \
  }

void parseCommand(char* command) {
  if (strcmp(command, "GET MAGNITUDES") == 0) {
    for (int i = 0; i < FFT_SIZE; ++i) {
      Serial.println(magnitudes[i]);
    }
  }
  else if (strcmp(command, "GET SAMPLES") == 0) {
    for (int i = 0; i < FFT_SIZE*2; i+=2) {
      Serial.println(samples[i]);
    }
  }
  else if (strcmp(command, "GET FFT_SIZE") == 0) {
    Serial.println(FFT_SIZE);
  }
  GET_AND_SET(SAMPLE_RATE_HZ)
  GET_AND_SET(TONE_ERROR_MARGIN_HZ)
  GET_AND_SET(TONE_WINDOW_MS)
  GET_AND_SET(TONE_THRESHOLD_DB)
}
