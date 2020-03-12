/**
 * Copyright (c) 2020 Todd Stellanova
 * License: BSD3
 *
 * Calibration tool for Bosch Sensortec / Hillcrest Labs bno080 AHRS.
 * Allows independent calibration of gyro, accel, mag.
 * You will need to connect the sensor to your arduino,
 * arduino to your host computer, and interact with the
 * arduino via the serial monitor.
 */
#include <Wire.h>
#include <SparkFun_BNO080_Arduino_Library.h>

BNO080 _bno;

//declare reset function at address 0
void(* resetFunc) (void) = 0;

void setup() {
  Serial.begin(9600);
  Serial.println();
  Serial.println("Calibration Tool");

  Wire.begin();
  // for i2c comms with bno080 we want 400 kHz clock
  Wire.setClock(400000);

  _bno.begin();

  Serial.println(F(" Calibration commands: "));
  Serial.println(F(" 'm' for mag calibration "));
  Serial.println(F(" 'l' for accel calibration "));
  Serial.println(F(" 'g' for gyro calibration "));
  Serial.println(F(" 'p' for planar accel calibration "));
//  Serial.println(F(" 'a' for calibration  all "));

}

bool save_calibration() {
  Serial.println(F("Saving calibration..."));
  // save the dynamic calibration data (DCD) to the sensor's flash
  _bno.saveCalibration();
  // obtain the current calibration status
  _bno.requestCalibrationStatus();
  while (true) {
    if (_bno.dataAvailable()) {
      if (_bno.calibrationComplete()) {
        Serial.println(F("Calibration saved!"));
        _bno.endCalibration();
        return true;
      }
    }
    delay(2);
  }

  return false;
}

bool calibrate_gyro() {
  Serial.println(F("Calibrating gyro..."));
  Serial.println(F("Leave sensor still on flat surface, Z up"));
  _bno.calibrateGyro();
  _bno.enableGyro(50);
  _bno.enableGameRotationVector(100);

  //collect data while user leaves sensor still
  delay(3000);
  while (true) {
    if (_bno.dataAvailable()) {
      uint8_t status = _bno.getGyroAccuracy();
      Serial.print(F("gyro status: "));
      Serial.println(status);
      if (status >= 3) {
        return save_calibration();
      }
      else {
        Serial.println(F("Calibration failed, try again"));
      }
      break;
    }
  }

  return calibrate_gyro();
}

bool calibrate_planar_accel() {
  Serial.println(F("Calibrating planar accel..."));
  Serial.println(F("Rotate sensor around Z axis"));
  _bno.calibratePlanarAccelerometer();
  _bno.enableAccelerometer(50);
  _bno.enableGameRotationVector(100);

  //collect data while user moves the sensor
  delay(3000);
  while (true) {
    if (_bno.dataAvailable()) {
      uint8_t status = _bno.getAccelAccuracy();
      Serial.print(F("accel status: "));
      Serial.println(status);
      if (status >= 3) {
        return save_calibration();
      }
      else {
        Serial.println(F("Calibration failed, try again"));
      }
      break;
    }
  }

  return calibrate_planar_accel();
}

bool calibrate_accel() {
  Serial.println(F("Calibrating 3D accel..."));
  _bno.calibrateAccelerometer();
  _bno.enableAccelerometer(50);
  _bno.enableGameRotationVector(100);

  Serial.println(F("Hold level with Z up "));
  delay(2500);
  Serial.println(F("Hold level with Z down "));
  delay(2500);
  Serial.println(F("Hold level with Y up "));
  delay(2500);
  Serial.println(F("Hold level with Y down "));
  delay(2500);
  Serial.println(F("Hold level with X down "));
  delay(2500);
  Serial.println(F("Hold level with X up "));
  delay(2500);

  while (true) {
    if (_bno.dataAvailable()) {
      uint8_t status = _bno.getAccelAccuracy();
      Serial.print(F("accel status: "));
      Serial.println(status);
      if (status >= 3) {
        return save_calibration();
      }
      else {
        Serial.println(F("Calibration failed, try again"));
      }
      break;
    }
  }

  return calibrate_accel();
}

bool calibrate_mag() {
  Serial.println(F("Calibrating mag..."));
  _bno.calibrateMagnetometer();
  _bno.enableMagnetometer(50);
  _bno.enableGameRotationVector(100);

  Serial.println(F("Rotate around X"));
  delay(3000);
  Serial.println(F("Rotate around Y"));
  delay(3000);
  Serial.println(F("Rotate around Z"));
  delay(3000);
  Serial.println(F("Figure 8 in XY plane"));
  delay(3000);

  while (true) {
    if (_bno.dataAvailable()) {
      uint8_t status = _bno.getMagAccuracy();
      Serial.print(F("mag status: "));
      Serial.println(status);
      if (status >= 3) {
        return save_calibration();
      }
      else {
        Serial.println(F("Calibration failed, try again"));
      }
      break;
    }
  }

  return calibrate_mag();
}

bool calibrate_all() {
  Serial.println(F("unimplemented!"));
  return false;
}

void loop() {

  if (Serial.available()) {
    int cmd = Serial.read();
    bool cal_performed = false;
    switch (cmd) {
      case 'm':
        cal_performed = calibrate_mag();
        break;
      case 'l':
        cal_performed = calibrate_accel();
        break;
      case 'p':
        cal_performed = calibrate_planar_accel();
        break;
      case 'g':
        cal_performed = calibrate_gyro();
        break;
      case 'a':
        cal_performed = calibrate_all();
        break;
      case -1:
        //no command available
        break;
    }
    if (cal_performed) {
      resetFunc();
    }
  }

}

