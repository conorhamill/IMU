// ---------------------------------------------------------------
// raw_data_4_imu.ino  (ESP32)
// Reads raw accelerometer and gyroscope data from FOUR MPU-9250s
// through a TCA9548A I2C multiplexer and prints one line per sample:
//
//    ax1,ay1,az1,gx1,gy1,gz1, ... ,ax4,ay4,az4,gx4,gy4,gz4
//
// (24 comma-separated values: IMU n = mux channel n-1)
//
// Wiring (ESP32 dev board, default I2C pins):
//    GPIO22 -> TCA9548A SCL
//    GPIO21 -> TCA9548A SDA
//    3V3    -> TCA9548A VIN and each IMU VCC  (ESP32 is 3.3 V logic)
//    GND    -> TCA9548A GND and each IMU GND
//    TCA9548A SD0/SC0 -> IMU 1 SDA/SCL
//    TCA9548A SD1/SC1 -> IMU 2 SDA/SCL
//    TCA9548A SD2/SC2 -> IMU 3 SDA/SCL
//    TCA9548A SD3/SC3 -> IMU 4 SDA/SCL
//    Each IMU AD0 -> GND (address 0x68 on its own channel)
// ---------------------------------------------------------------

#include <Wire.h>

const int MUX_ADDR = 0x70;      // TCA9548A (A0-A2 low)
const int MPU_ADDR = 0x68;      // every IMU (AD0 low, one per channel)

// MPU-9250 register addresses (from the datasheet / register map)
const int PWR_MGMT_1   = 0x6B;  // power management register
const int ACCEL_XOUT_H = 0x3B;  // first of 14 data registers

const int NUM_IMUS = 4;         // on mux channels 0, 1, 2, 3

// Open one mux channel (and close the others) by writing a byte
// with the matching bit set to the TCA9548A.
void muxSelect(int channel) {
  Wire.beginTransmission(MUX_ADDR);
  Wire.write(1 << channel);
  Wire.endTransmission();
}

void setup() {
  Serial.begin(115200);  // fast baud rate so we can send data quickly
  Wire.begin();          // I2C master on default pins (SDA 21, SCL 22)

  // The MPU-9250 starts up in sleep mode.
  // Wake each one by writing 0 to its power management register.
  for (int i = 0; i < NUM_IMUS; i++) {
    muxSelect(i);
    Wire.beginTransmission(MPU_ADDR);
    Wire.write(PWR_MGMT_1);
    Wire.write(0);
    Wire.endTransmission();
  }
}

void loop() {
  // Read all four IMUs in turn and build one long line.
  for (int i = 0; i < NUM_IMUS; i++) {
    muxSelect(i);

    // The 14 data registers starting at 0x3B hold, in order:
    //   accel X, Y, Z  (2 bytes each, high byte first)
    //   temperature    (2 bytes - we read it but ignore it)
    //   gyro X, Y, Z   (2 bytes each, high byte first)
    Wire.beginTransmission(MPU_ADDR);
    Wire.write(ACCEL_XOUT_H);
    Wire.endTransmission(false);   // false = keep the bus open for the read

    Wire.requestFrom(MPU_ADDR, 14);

    int16_t ax = (Wire.read() << 8) | Wire.read();
    int16_t ay = (Wire.read() << 8) | Wire.read();
    int16_t az = (Wire.read() << 8) | Wire.read();
    int16_t temp = (Wire.read() << 8) | Wire.read();  // unused
    int16_t gx = (Wire.read() << 8) | Wire.read();
    int16_t gy = (Wire.read() << 8) | Wire.read();
    int16_t gz = (Wire.read() << 8) | Wire.read();

    Serial.print(ax); Serial.print(',');
    Serial.print(ay); Serial.print(',');
    Serial.print(az); Serial.print(',');
    Serial.print(gx); Serial.print(',');
    Serial.print(gy); Serial.print(',');
    Serial.print(gz);
    if (i < NUM_IMUS - 1) Serial.print(',');
  }
  Serial.println();

  delay(50);  // ~20 samples per second - plenty for a display
}
