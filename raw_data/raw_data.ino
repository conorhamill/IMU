// ---------------------------------------------------------------
// raw_data.ino
// Reads raw accelerometer and gyroscope data from an MPU-9250
// over I2C and prints it to the serial port as one line per sample:
//
//    ax,ay,az,gx,gy,gz
//
// Wiring (Arduino Uno/Nano):
//    SCL -> A5
//    SDA -> A4
//    AD0 -> GND  (this sets the I2C address to 0x68)
//    VCC -> 3.3V (most breakout boards also accept 5V - check yours)
//    GND -> GND
// ---------------------------------------------------------------

#include <Wire.h>

// I2C address of the MPU-9250 (0x68 because AD0 is pulled low)
const int MPU_ADDR = 0x68;

// MPU-9250 register addresses (from the datasheet / register map)
const int PWR_MGMT_1   = 0x6B;  // power management register
const int ACCEL_XOUT_H = 0x3B;  // first of 14 data registers

void setup() {
  Serial.begin(115200);  // fast baud rate so we can send data quickly
  Wire.begin();          // join the I2C bus as master

  // The MPU-9250 starts up in sleep mode.
  // Writing 0 to the power management register wakes it up.
  Wire.beginTransmission(MPU_ADDR);
  Wire.write(PWR_MGMT_1);
  Wire.write(0);
  Wire.endTransmission();
}

void loop() {
  // The 14 data registers starting at 0x3B hold, in order:
  //   accel X, Y, Z  (2 bytes each, high byte first)
  //   temperature    (2 bytes - we read it but ignore it)
  //   gyro X, Y, Z   (2 bytes each, high byte first)

  // Tell the sensor which register we want to start reading from
  Wire.beginTransmission(MPU_ADDR);
  Wire.write(ACCEL_XOUT_H);
  Wire.endTransmission(false);   // false = keep the bus open for the read

  // Request all 14 bytes in one go
  Wire.requestFrom(MPU_ADDR, 14);

  // Each value is 16 bits: shift the high byte left 8 bits,
  // then OR in the low byte. int16_t keeps the sign correct.
  int16_t ax = (Wire.read() << 8) | Wire.read();
  int16_t ay = (Wire.read() << 8) | Wire.read();
  int16_t az = (Wire.read() << 8) | Wire.read();
  int16_t temp = (Wire.read() << 8) | Wire.read();  // unused
  int16_t gx = (Wire.read() << 8) | Wire.read();
  int16_t gy = (Wire.read() << 8) | Wire.read();
  int16_t gz = (Wire.read() << 8) | Wire.read();

  // Print as comma-separated values so Python can split them easily
  Serial.print(ax); Serial.print(',');
  Serial.print(ay); Serial.print(',');
  Serial.print(az); Serial.print(',');
  Serial.print(gx); Serial.print(',');
  Serial.print(gy); Serial.print(',');
  Serial.println(gz);

  delay(50);  // ~20 samples per second - plenty for a display
}
