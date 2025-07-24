#include <Arduino.h>

#include <Wire.h>
 
/**********************************************************************  VARIABLES DEFINITION ***********************************************************************************/

/********************************************************************************************************************************************************************************/
 
// Defining addresses of sensors

#define ADD0 0b00

#define ADD1 0b10 // Sensor 1 and 2 have their addresses swapped because of a connection error

#define ADD2 0b01

#define ADD3 0b11

#define sensor_base 0b1100
 
// Defining config modes (every sensor will have same config)

#define BIST 0

#define Z_SERIES 0

#define GAIN_SEL 5

#define HALLCONF 0xC

#define TRIG_INT 1

#define COMM 3

#define WOC_DIFF 0

#define EXT_TRIG 0

#define TCOMP 0

#define BURST_SEL 0b1110

#define BURST_DATA_RATE 1

#define OSR2 0

#define RESXYZ 0b010101

#define DIG_FILT 5

#define OSR 0
 
const byte sensors_array[] = {ADD0, ADD1, ADD2, ADD3};
 
#define CalMeasurements 50

#define Threshold 50  //
 
int16_t offsets[4][3] = {0};    // 

int32_t calBuffer[4][3] = {0};  // 

int calCount = 0;

bool touchDetected = false;
 
/********************************************************************************************************************************************************************************/

/********************************************************************************************************************************************************************************/
 
void WireFlush() {

  while (Wire.available() > 0) {

    Wire.read();

  }

}
 
void Measure(byte address, int16_t *measurements) {

  uint8_t data[7];

  Wire.beginTransmission(address);

  Wire.write((byte)0x4E);

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
 
void WriteAllRegisters(byte BH, byte BL, byte A) {

  for (int i = 0; i < 4; i++) {

    WriteRegister(sensor_base | sensors_array[i], BH, BL, A);

  }

}
 
void ExitAll() {

  for (int i = 0; i < 4; i++) {

    Wire.beginTransmission(sensor_base | sensors_array[i]);

    Wire.write(byte(0x80));

    Wire.endTransmission(1);

  }

  delay(250);

}
 
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

  delay(250);

}
 
void ResetAll() {

  for (int i = 0; i < 4; i++) {

    Wire.beginTransmission(sensor_base | sensors_array[i]);

    Wire.write(byte(0xF0));

    Wire.endTransmission(1);

  }

  delay(250);

}
 
void StartBurst() {

  for (int i = 0; i < 4; i++) {

    Wire.beginTransmission(sensor_base | sensors_array[i]);

    Wire.write(0b00011110); // burst mode, zyx

    Wire.endTransmission(0);

    Wire.requestFrom(sensor_base | sensors_array[i], 1, 1);

    Wire.endTransmission(1);

    if (Wire.available()) Wire.read();

  }

  delay(100);

}
 
// 

void StaticCalibration() {

  int32_t sum[4][3] = {0};
 
  Serial.println("Starting static calibration. Please keep sensors still...");
 
  for (int s = 0; s < CalMeasurements; s++) {

    for (int i = 0; i < 4; i++) {

      int16_t measurement[3];

      Measure(sensor_base | sensors_array[i], measurement);

      for (int j = 0; j < 3; j++) {

        sum[i][j] += measurement[j];

      }

    }

    delay(100);

  }
 
  for (int i = 0; i < 4; i++) {

    for (int j = 0; j < 3; j++) {

      offsets[i][j] = sum[i][j] / CalMeasurements;

    }

  }
 
  Serial.println("Static calibration completed.");

}
 
// 

bool checkTouch(int16_t measurements[4][3]) {

  for (int i = 0; i < 4; i++) {

    if (abs(measurements[i][2] - offsets[i][2]) > Threshold) {

      return true;

    }

  }

  return false;

}
 
/********************************************************************** SETUP ******************************************************************/

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
 
  StaticCalibration(); // 

}
 
/********************************************************************** LOOP *******************************************************************/

void loop() {

  int16_t measurements[4][3];
 
  // 

  for (int i = 0; i < 4; i++) {

    Measure(sensor_base | sensors_array[i], measurements[i]);

  }
 
  bool currentTouch = checkTouch(measurements);
 
  if (!currentTouch) {

    // 

    if (calCount == 0) {

      // 

      for (int i = 0; i < 4; i++)

        for (int j = 0; j < 3; j++)

          calBuffer[i][j] = 0;

    }

    // 

    for (int i = 0; i < 4; i++) {

      for (int j = 0; j < 3; j++) {

        calBuffer[i][j] += measurements[i][j];

      }

    }

    calCount++;
 
    // 

    if (calCount >= CalMeasurements) {

      for (int i = 0; i < 4; i++) {

        for (int j = 0; j < 3; j++) {

          offsets[i][j] = calBuffer[i][j] / CalMeasurements;

        }

      }

      calCount = 0;

      Serial.println("Dynamic calibration updated offsets.");

    }

  } else {

    // 

    calCount = 0;

  }
 
  // 

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
 
  delay(100); // 

}

 
