/*
  Read MyoWare Voltage Example Code
  Advancer Technologies, LLC
  Brian Kaminski
  1/12/2024

  This example reads a MyoWare 2.0 Muscle Sensor output on A0-A3 where A0 is ENV,
  A1 is RAW, A2 is RECT, and A3 is REF. It then converts the reading to the amplitude of the 
  muscle activity as it appears at the electrodes in millivolts. 
  
  MyoWare Muscle Sensor Analog Output:
  1. Raw EMG Output (RAW) - This is the raw amplified and filtered output:
  * We will first remove the DC voltage offset using the REF value, converts its
  value to volts based on the ADC parameters, and remove the gain applied by the
  sensor using the RAW gain equation which is fixed at 200.
  
  2. Rectified EMG Output (RECT) - This is the full-ware rectified RAW output:
  * We will first convert its value to volts based on the ADC parameters and remove
  the gain applied by the sensor using the RAW gain equation which is fixed at 200.
  
  3. EMG Envelope (ENV) - This is the amplified envelope of the RECT output:
  * We will first convert its value to volts based on the ADC parameters and remove
  the gain applied the sensor using the ENV gain equation, see below. ENV has an second
  amplification stage which is adjustable using the gain potentiometer. We will need
  the gain potentiometer's resistance in kOhms to calcuate the gain.

  Read more about the MyoWare 2.0 Muscle Sensor & electromyography (EMG) output here:
  https://myoware.com/learn/tutorials-guides/

   In order for this example to work, you will need a MyoWare 2.0 Muscle Sensor
   with the Vin and GND pins connected to 5V and GND pins on an Arduino compatible
   board. The ENV, RAW, and REF pins will need to connect to the A0, A1, and A2 pins
   on the Arduino compatible board, respectively.

  Hardware:
  MyoWare 2.0 Muscle Sensor
  Arduino compatible board (e.g Uno, Mega, etc.)
  USB Cable

  Graphical representation is available using Serial Plotter (Tools > Serial Plotter menu).

  This example code is in the public domain.
*/

#include <MyoWare.h>

// MyoWare class object
// MyoWare myoware;

// the setup routine runs once when you press reset:
int initialize_emg(MyoWare* emg, int env_pin) 
{
  // initialize serial communication at 9600 bits per second:
  // Serial.begin(9600);

  // output conversion parameters - modify these values to match your setup
  emg->setConvertOutput(true);     // Set to true to convert ADC output to the amplitude of
                                      // of the muscle activity as it appears at the electrodes
                                      // in millivolts
  emg->setADCResolution(12.);      // ADC bits (shield default = 12-bit)
  emg->setADCVoltage(3.3);         // ADC reference voltage (shield default = 3.3V)
  emg->setGainPotentiometer(50.);  // Gain potentiometer resistance in kOhms.
                                      // adjust the potentiometer setting such that the
                                      // max muscle reading is below 3.3V then update this
                                      // parameter to the measured value of the potentiometer
  emg->setENVPin(env_pin);              // Arduino pin connected to ENV
  // emg->setRAWPin(A0);              // Arduino pin connected to RAW
  // emg->setREFPin(A1);              // Arduino pin connected to REF
  // emg->setRECTPin(A0);             // Arduino pin connected to RECT

  return 0;
}

// the loop routine runs over and over again forever:
float* read_emg(MyoWare* emg, float* packet, int packet_position) 
{
  const double envMillivolts = emg->readSensorOutput(MyoWare::ENVELOPE);
  // const double rectMillivolts = emg->readSensorOutput(MyoWare::RECTIFIED);
  packet[packet_position] = (float) envMillivolts * 10000;

  return packet;
}
