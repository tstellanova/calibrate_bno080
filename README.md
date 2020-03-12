# calibrate_bno080

This is a simple Arduino-based tool that
communicates with a BNO080 AHRS via i2c and
calibrates its various sensors.

## Usage 
- Connect Arduino to host computer so that you may use the Serial Monitor tool
- Connect BNO080 to Arduino via i2c.  Ensure that the BNO080 you're using has pull-up resistors on the SCL and SDA lines 
(or that the Arduino does)
- Follow the text instructions in the Serial Monitor
