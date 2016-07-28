/*  Copyright (C) 2015 Alessandro Tondo
 *  email: tondo.codes+ros <at> gmail.com
 *
 *  This program is free software: you can redistribute it and/or modify it under the terms of the GNU General Public
 *  License as published by the Free Software Foundation, either version 3 of the License, or (at your option) any
 *  later version.
 *  This program is distributed in the hope that it will be useful, but WITHOUT ANY WARRANTY; without even the implied
 *  warranty of MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the GNU General Public License for more
 *  details.
 *  You should have received a copy of the GNU General Public License along with this program.
 *  If not, see <http://www.gnu.org/licenses/>.
 *
 *
 *  This sketch reads data from 5 IMUs (MPU-9250) using an Arduino Micro
 */

#include <Wire.h>

// For registers not listed below see 'MPU-9250 Register Map and Descriptions', RM-MPU-9250A-00, Rev. 1.4, 9/9/2013
#define GYRO_CONFIG              0x1B
#define ACCEL_CONFIG             0x1C
#define MPU9250_PWR_MGMT_1       0x6B
#define MPU9250_I2C_ADDRESS      0x69
#define MPU9250_ACCEL_XOUT_H     0x3B

uint8_t dummy;
#define SWITCH(x,y) dummy = x; x = y; y = dummy  // exchanges bytes in a 16 bit word (lower_byte <-> higher_byte)

typedef union accel_t_gyro_union {
  struct {
    uint8_t x_accel_h;
    uint8_t x_accel_l;
    uint8_t y_accel_h;
    uint8_t y_accel_l;
    uint8_t z_accel_h;
    uint8_t z_accel_l;
    uint8_t t_h;
    uint8_t t_l;
    uint8_t x_gyro_h;
    uint8_t x_gyro_l;
    uint8_t y_gyro_h;
    uint8_t y_gyro_l;
    uint8_t z_gyro_h;
    uint8_t z_gyro_l;
  } reg;
  struct {
    int16_t x_accel;
    int16_t y_accel;
    int16_t z_accel;
    int16_t temperature;
    int16_t x_gyro;
    int16_t y_gyro;
    int16_t z_gyro;
  } value;
};

const int num_imus = 5;
int pin_base = 4;  // all IMUs must be attached to consecutive pins, starting from pin_base

int vect_acc[num_imus][3];
int vect_gyr[num_imus][3];
// TODO: future improvement int vect_mag[num_imus][3];

int count_samples = 0;
unsigned long start_time = 0;

int error = 0;  // errors handle
int input = 0;  // input handle
int skip = 0;
byte comm_status = 0x00;  /*  communication status
                           *  byte: x000|00ga
                           *  x: 1 -> reset communication after 1 step
                           *  a: 1 -> acceleration data
                           *  g: 1 -> gyro data
                           */

void setup() {
  for (int i=0; i<num_imus; i++) {
    pinMode(pin_base + i, OUTPUT);
  }

  // starts I2C and serial communications
  Wire.begin();
  Serial.begin(115200);

  for (int i=0; i<num_imus; i++) {
    // selects one of the IMUs to set its parameters
    for (int j=0; j<num_imus; j++) {
      if (j == i)
        digitalWrite(j + pin_base, HIGH);  // i=0 -> pin4=HIGH
      else
        digitalWrite(j + pin_base, LOW);  // others LOW
    }
    delay(50);

    error = MPU9250_write_reg(MPU9250_PWR_MGMT_1, 0);
    if (error != 0) {
      Serial.print("ERROR: can't configure IMU attached to pin ");
      Serial.println(pin_base + i);
      return;
    }
    delay(2);

    error = MPU9250_write_reg(ACCEL_CONFIG, 0x10);    // 0x00 2g, 0x08 4g, 0x10 8g, 0x18 16g
    if (error != 0) {
      Serial.print("ERROR: can't configure IMU attached to pin ");
      Serial.println(pin_base + i);
      return;
    }
    delay(2);

    error = MPU9250_write_reg(GYRO_CONFIG, 0x10);    // 0x00 250dps, 0x08 500dps, 0x10 1000dps, 0x18 2000dps
    if (error != 0) {
      Serial.print("ERROR: can't configure IMU attached to pin ");
      Serial.println(pin_base + i);
      return;
    }
    delay(2);

    for (int j=0; j<3; j++) {
      vect_acc[i][j] = 0;
      vect_gyr[i][j] = 0;
    }

    delay(200);
  }
}

void loop() {
  if (Serial.available() > 0) {
    input = Serial.read();
    
    // starts communication to get only acceleration data
    if (input == 'A') {
      comm_status = 0x01;
      count_samples = 0;
      start_time = millis();
    }
    // starts communication to get one sample of only acceleration data
    else if (input == 'a') {
      comm_status = 0x81;
    }
    // starts communication to get only gyro data
    else if (input == 'G') {
      comm_status = 0x02;
      count_samples = 0;
      start_time = millis();
    }
    // starts communication to get one sample of only gyro data
    else if (input == 'g') {
      comm_status = 0x82;
    }
    // starts communication to get acceleration and gyro data
    else if (input == '*') {
      comm_status = 0x03;
      count_samples = 0;
      start_time = millis();
    }
    // starts communication to get one sample of acceleration and gyro data
    else if (input == '+') {
      comm_status = 0x83;
    }
    // stops communication and prints statistics
    else if (input == '!') {
      comm_status = 0x00;
      Serial.println(" ");
      Serial.print("Sample rate: ");
      Serial.println((double)count_samples / ((double)(millis() - start_time) / 1000));
      Serial.println(" ");
    }
  }

  // reads from sensors in a polling fashion
  for (int i=0; i<num_imus; i++) {
    // selects one of the IMUs to get its data values
    for (int j=0; j<num_imus; j++) {
      if (j == i)
        digitalWrite(j + pin_base, HIGH);  // i=0 -> pin4=HIGH
      else
        digitalWrite(j + pin_base, LOW);  // others LOW
    }
    delayMicroseconds(200);

    accel_t_gyro_union accel_t_gyro;
    error = MPU9250_read(MPU9250_ACCEL_XOUT_H, (uint8_t *) &accel_t_gyro, sizeof(accel_t_gyro));
    if (error == 0) {  // data retrieved correctly
      // switches lower and upper 8 bit registers and stores data in the proper array
      SWITCH(accel_t_gyro.reg.x_accel_h, accel_t_gyro.reg.x_accel_l);
      SWITCH(accel_t_gyro.reg.y_accel_h, accel_t_gyro.reg.y_accel_l);
      SWITCH(accel_t_gyro.reg.z_accel_h, accel_t_gyro.reg.z_accel_l);
      SWITCH(accel_t_gyro.reg.t_h, accel_t_gyro.reg.t_l);
      SWITCH(accel_t_gyro.reg.x_gyro_h, accel_t_gyro.reg.x_gyro_l);
      SWITCH(accel_t_gyro.reg.y_gyro_h, accel_t_gyro.reg.y_gyro_l);
      SWITCH(accel_t_gyro.reg.z_gyro_h, accel_t_gyro.reg.z_gyro_l);
      
      vect_acc[i][0] = accel_t_gyro.value.x_accel;
      vect_acc[i][1] = accel_t_gyro.value.y_accel;
      vect_acc[i][2] = accel_t_gyro.value.z_accel;
      vect_gyr[i][0] = accel_t_gyro.value.x_gyro;
      vect_gyr[i][1] = accel_t_gyro.value.y_gyro;
      vect_gyr[i][2] = accel_t_gyro.value.z_gyro;
    }
    else {  // data error
      vect_acc[i][0] = 0;
      vect_acc[i][1] = 0;
      vect_acc[i][2] = 0;
      vect_gyr[i][0] = 0;
      vect_gyr[i][1] = 0;
      vect_gyr[i][2] = 0;
    }
  }
  count_samples++;
  
  // outputs data on the serial communication (only if communication is enabled)
  char str[8];
  if (comm_status > 0) {
    // if command is '*' the communication has to be slowed due to the saturation of the serial
    if (comm_status & 0x01 && comm_status & 0x02 && !(comm_status & 0x80)) {  // command '*'
      skip++;
      skip %= 2;
    }
    else { 
      skip = 0;
    }

    if (!skip) {  // skips only when command '*' is enabled
      if (comm_status & 0x01) {  // commands 'A', 'a', '*', '+'
        // sends the whole acquisition in one line, each number separated by ';'
        for (int i = 0; i < 5; i++) {
          for (int j = 0; j < 3; j++) {
            sprintf(str, "%06d;", vect_acc[i][j]);
            Serial.print(str);
          }
        }
        Serial.print("\n");
      }
      if (comm_status & 0x02) {  // commands 'G', 'g', '*', '+'
        // sends the whole acquisition in one line, each number separated by ';'
        for (int i = 0; i < 5; i++) {
          for (int j = 0; j < 3; j++) {
            sprintf(str, "%06d;", vect_gyr[i][j]);
            Serial.print(str);
          }
        }
        Serial.print("\n");
      }
    }

    // stops the communication in case of commands 'a', 'g', '+'
    if (comm_status & 0x80)
      comm_status = 0x00;
  }
}


int MPU9250_read(int start, uint8_t *buffer, int size) {
  int i, n;

  Wire.beginTransmission(MPU9250_I2C_ADDRESS);
  n = Wire.write(start);
  if (n != 1)
    return -10;

  n = Wire.endTransmission(false);  // holds the I2C-bus
  if (n != 0)
    return n;

  // third parameter is true: relases I2C-bus after data is read
  Wire.requestFrom(MPU9250_I2C_ADDRESS, size, true);
  i = 0;
  while (Wire.available() && i < size) {
    buffer[i++] = Wire.read();
  }
  if (i != size)
    return -11;

  return 0;  // no errors
}

int MPU9250_write(int start, const uint8_t *pData, int size) {
  int n, error;

  Wire.beginTransmission(MPU9250_I2C_ADDRESS);
  n = Wire.write(start);
  if (n != 1)
    return -20;

  n = Wire.write(pData, size);
  if (n != size)
    return -21;

  error = Wire.endTransmission(true);  // releases the I2C-bus
  if (error != 0)
    return error;

  return 0;  // no errors
}

int MPU9250_write_reg(int reg, uint8_t data) {
  return MPU9250_write(reg, &data, 1);
}

