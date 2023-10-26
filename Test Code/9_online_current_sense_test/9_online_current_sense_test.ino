// MKS MINI FOC; Online Current sense test; Test Hardware：MKS MINI FOC V1.0
// The data measured by this example is the real-time current of the three phase wires of the motor.
// Display sampled data in serial monitor.
// Display real-time data plots in a serial plotter.

#include <SimpleFOC.h>

// Current Detection
// Sampling Resistor Value Gain ADC Pin
InlineCurrentSense current_sense0 = InlineCurrentSense(0.01, 50.0, 39, 36);
InlineCurrentSense current_sense1 = InlineCurrentSense(0.01, 50.0, 35, 34);


void setup() {
  // Current Detection
  current_sense0.init();
  current_sense1.init();

  current_sense0.gain_b *= -1;
  current_sense1.gain_b *= -1;
  
  Serial.begin(115200);
  Serial.println("Current sense ready.");
}

void loop() {

  PhaseCurrent_s currents0 = current_sense0.getPhaseCurrents();
  float current_magnitude0 = current_sense0.getDCCurrent();
  PhaseCurrent_s currents1 = current_sense1.getPhaseCurrents();
  float current_magnitude1 = current_sense1.getDCCurrent();

  Serial.print(currents0.a*1000); // milli Amps
  Serial.print("\t");
  Serial.print(currents0.b*1000); // milli Amps
  Serial.print("\t");
  Serial.print(currents0.c*1000); // milli Amps
  Serial.print("\t");
  Serial.println(current_magnitude0*1000); // milli Amps
  Serial.print(currents1.a*1000); // milli Amps
  Serial.print("\t");
  Serial.print(currents1.b*1000); // milli Amps
  Serial.print("\t");
  Serial.print(currents1.c*1000); // milli Amps
  Serial.print("\t");
  Serial.println(current_magnitude1*1000); // milli Amps
  Serial.println();
}
