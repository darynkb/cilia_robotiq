#include <Arduino.h>
#include <Wire.h>
 
/**********************************************************************  VARIABLES DEFINITION ***********************************************************************************/
 
// Define sensor addresses. Note: Sensor 1 and 2 have swapped addresses due to connection error.
#define ADD0 0b00
#define ADD1 0b10 // Sensor 1 and 2 addresses are swapped
#define ADD2 0b01
#define ADD3 0b11
#define sensor_base 0b1100 // Base I2C address for all sensors
 
// Configuration modes (same for all sensors)
#define BIST 0            // Built-in self test (off)
#define Z_SERIES 0        // Z-axis series mode (off)
#define GAIN_SEL 5        // Gain selection
#define HALLCONF 0xC      // Hall sensor configuration
#define TRIG_INT 1        // Internal trigger
#define COMM 3            // Communication mode
#define WOC_DIFF 0        // Wake-on-change differential (off)
#define EXT_TRIG 0        // External trigger (off)
#define TCOMP 0           // Temperature compensation (off)
#define BURST_SEL 0b1110  // Burst selection
#define BURST_DATA_RATE 1 // Burst data rate
#define OSR2 0            // Oversampling rate 2
#define RESXYZ 0b010101   // Resolution for XYZ axes
#define DIG_FILT 7        // Digital filter setting
#define OSR 0             // Oversampling rate
 
const byte sensors_array[] = {ADD0, ADD1, ADD2, ADD3}; // Array of sensor address offsets
 
#define CalMeasurements 60 // Number of measurements for calibration
#define Threshold 60      // Threshold for detecting touch
 
int16_t offsets[4][3] = {0};    // Offset values for each sensor and axis (XYZ)
int32_t calBuffer[4][3] = {0};  // Buffer for accumulating calibration measurements
int calCount = 0;               // Counter for calibration measurements
unsigned long prevMicros = 0;
 
/**********************************************************************  HELPER FUNCTIONS ***********************************************************************************/
 
// Flush the I2C buffer
void WireFlush() {
  while (Wire.available() > 0) {
    Wire.read();
  }
}
 
// Read measurements from a sensor
void Measure(byte address, int16_t *measurements) {
  uint8_t data[7];
  Wire.beginTransmission(address);
  Wire.write((byte)0x4E); // Request measurement
  Wire.endTransmission((uint8_t)0);
  Wire.requestFrom((uint8_t)address, (uint8_t)7, (uint8_t)1);
 
  if (Wire.available() == 7) {
    for (int i = 0; i < 7; i++) data[i] = Wire.read();
    measurements[0] = (data[1] << 8) | data[2]; // X
    measurements[1] = (data[3] << 8) | data[4]; // Y
    measurements[2] = (data[5] << 8) | data[6]; // Z
  }
  WireFlush();
}
 
// Write a value to a register on a device
void WriteRegister(byte deviceAddress, byte regAddressHigh, byte regAddressLow, byte value) {
  Wire.beginTransmission(deviceAddress);
  Wire.write(regAddressHigh);
  Wire.write(regAddressLow);
  Wire.write(value);
  Wire.endTransmission();
  delay(10);
  Wire.requestFrom((uint8_t)deviceAddress, (uint8_t)1, (uint8_t)1);
  if (Wire.available()) Wire.read();
}
 
// Write a value to all registers (for all sensors)
void WriteAllRegisters(byte BH, byte BL, byte A) {
  for (int i = 0; i < 4; i++) {
    WriteRegister(sensor_base | sensors_array[i], BH, BL, A);
  }
}
 
// Exit all sensors (put in standby)
void ExitAll() {
  for (int i = 0; i < 4; i++) {
    Wire.beginTransmission(sensor_base | sensors_array[i]);
    Wire.write(byte(0x80));
    Wire.endTransmission(1);
  }
  delay(50);
}
 
// Configure all sensors with the above parameters
void ConfigureSensors() {
  byte BH = 0x00 | BIST;
  byte BL = 0x00 | (Z_SERIES << 7);
  BL = BL | (GAIN_SEL << 4) | HALLCONF;
  WriteAllRegisters(BH, BL, 0);
 
  BH = 0x00 | (TRIG_INT << 7) | (COMM << 5) | (WOC_DIFF << 4) | (EXT_TRIG << 3) | (TCOMP << 2) | (BURST_SEL >> 2);
  BL = (uint8_t)(BURST_SEL << 6) | BURST_DATA_RATE;
  WriteAllRegisters(BH, BL, 1);
 
  BH = 0x00 | (OSR2 << 3) | (RESXYZ >> 3);
  BL = ((RESXYZ << 5) & 0xFF) | (DIG_FILT << 2) | OSR;
  WriteAllRegisters(BH, BL, 2);
 
  ExitAll();
  WireFlush();
  delay(50);
}
 
// Reset all sensors
void ResetAll() {
  for (int i = 0; i < 4; i++) {
    Wire.beginTransmission(sensor_base | sensors_array[i]);
    Wire.write(byte(0xF0));
    Wire.endTransmission(1);
  }
  delay(50);
}
 
// Start burst mode (continuous measurement) on all sensors
void StartBurst() {
  for (int i = 0; i < 4; i++) {
    Wire.beginTransmission(sensor_base | sensors_array[i]);
    Wire.write(0b00011110); // burst mode, zyx
    Wire.endTransmission(0);
    Wire.requestFrom(sensor_base | sensors_array[i], 1, 1);
    Wire.endTransmission(1);
    if (Wire.available()) Wire.read();
  }
  delay(50);
}
 
/**********************************************************************  CALIBRATION FUNCTIONS ***********************************************************************************/
 
// Perform static calibration (initial calibration with sensors at rest)
//void StaticCalibration() {
//  int32_t sum[4][3] = {0};
//
//  Serial.println("Starting static calibration. Please keep sensors still...");
//
//  int startIndex = 20; // Skip first 20 samples to avoid transient effects
//  int validSamples = CalMeasurements - startIndex;
//
//  for (int s = 0; s < CalMeasurements; s++) {
//    for (int i = 0; i < 4; i++) {
//      int16_t measurement[3];
//      Measure(sensor_base | sensors_array[i], measurement);
//
//      if (s >= startIndex) {
//        for (int j = 0; j < 3; j++) {
//          sum[i][j] += measurement[j]; // Accumulate valid samples
//        }
//      }
//    }
//    delay(300); // Wait between samples
//  }
//
//  for (int i = 0; i < 4; i++) {
//    for (int j = 0; j < 3; j++) {
//      offsets[i][j] = sum[i][j] / validSamples; // Calculate average as offset
//    }
//  }
//
//  Serial.println("Static calibration completed.");
//}
 
 
void StaticCalibration() {
  int32_t sum[4][3] = {0};
  int validSamples[4] = {0, 0, 0, 0}; // Track valid samples per sensor
 
  Serial.println("Starting static calibration. Please keep sensors still...");
 
  int startIndex = 50; // Skip first 20 samples
  int totalSamples = CalMeasurements;
 
  for (int s = 0; s < totalSamples; s++) {
    for (int i = 0; i < 4; i++) {
      int16_t measurement[3] = {0};
      int retries = 0;
      bool valid = false;
 
      // Try up to 5 times to get a valid reading
      while (retries < 5) {
        Measure(sensor_base | sensors_array[i], measurement);
        if (measurement[0] != 0 || measurement[1] != 0 || measurement[2] != 0) {
          valid = true;
          break;
        }
        retries++;
        delay(200); // Wait before retry
      }
 
      // Only accumulate if this is a valid reading and after the initial transient phase
      if (valid && s >= startIndex) {
        for (int j = 0; j < 3; j++) {
          sum[i][j] += measurement[j];
        }
        validSamples[i]++;
//      }else {
//        Serial.println("Couldnt initialize sensor taxel number ");
//        Serial.println(i);
      }
    }
    delay(250); // Increased delay for sensor stability
  }
 
  // Calculate offsets using only valid samples
  for (int i = 0; i < 4; i++) {
    for (int j = 0; j < 3; j++) {
      if (validSamples[i] > 0) {
        offsets[i][j] = sum[i][j] / validSamples[i];
      } else {
        offsets[i][j] = 0; // Fallback if no valid samples
      }
    }
  }
 
  Serial.println("Static calibration completed.");
}
 
 
// Check if any sensor detects a touch (Z-axis exceeds threshold)
bool checkTouch(int16_t measurements[4][3]) {
  for (int i = 0; i < 4; i++) {
    if (abs(measurements[i][2] - offsets[i][2]) > Threshold) {
      return true;
    }
  }
  return false;
}
 
/**********************************************************************  SETUP ******************************************************************/
void setup() {
  Serial.begin(115200);
  Wire.begin();
  digitalWrite(SDA, 0);
  digitalWrite(SCL, 0);
 
  ExitAll();
  ResetAll();
  ConfigureSensors();
  StartBurst();
  delay(100);
 
  StaticCalibration(); // Perform initial static calibration
}
 
/**********************************************************************  LOOP *******************************************************************/
void loop() {
  unsigned long now = micros();
  unsigned long delta = now - prevMicros;
  prevMicros = now;
  int16_t measurements[4][3];
 
  // Read measurements from all sensors
  for (int i = 0; i < 4; i++) {
    Measure(sensor_base | sensors_array[i], measurements[i]);
  }
 
  bool currentTouch = checkTouch(measurements);
 
  if (!currentTouch) {
    // If no touch detected, accumulate measurements for dynamic calibration
    if (calCount == 0) {
      // Reset buffer if starting new calibration cycle
      for (int i = 0; i < 4; i++)
        for (int j = 0; j < 3; j++)
          calBuffer[i][j] = 0;
    }
 
    for (int i = 0; i < 4; i++) {
      for (int j = 0; j < 3; j++) {
        calBuffer[i][j] += measurements[i][j];
      }
    }
    calCount++;
 
    if (calCount >= CalMeasurements) {
      // Update offsets if enough measurements collected
      for (int i = 0; i < 4; i++) {
        for (int j = 0; j < 3; j++) {
          offsets[i][j] = calBuffer[i][j] / CalMeasurements;
        }
      }
      calCount = 0;
      Serial.println("Dynamic calibration updated offsets.");
    }
  } else {
    // If touch detected, reset calibration counter
    calCount = 0;
  }
 
  // Print measurements (offset-corrected)
  static bool printedHeader = false;
  if (!printedHeader) {
    Serial.println("X1\tY1\tZ1\tX2\tY2\tZ2\tX3\tY3\tZ3\tX4\tY4\tZ4");
    printedHeader = true;
  }
 
  for (int i = 0; i < 4; i++) {
    Serial.print(measurements[i][0] - offsets[i][0]); Serial.print("\t");
    Serial.print(measurements[i][1] - offsets[i][1]); Serial.print("\t");
    Serial.print(measurements[i][2] - offsets[i][2]);
    if (i < 3) Serial.print("\t");
  }
  Serial.println();
  Serial.print("Loop time (µs): ");
  Serial.println(delta);
 
//  delay(100);/
}
