/*************************************************
 * Includes
 *************************************************/
#include <Wire.h>
#include <math.h>

/*************************************************
 * Variables
 *************************************************/
int gyro_x, gyro_y, gyro_z; // Raw data from gyroscope.
long acc_x, acc_y, acc_z; // Raw data from accelerometer.
long gyro_x_cal, gyro_y_cal, gyro_z_cal; //Calibrated gyroscope values.
float angle_pitch_gy, angle_roll_gy, angle_yaw_gy; // Calculated angle from gyro data.
float angle_roll_acc, angle_pitch_acc, angle_yaw_acc; // Calculated acceleration angle from accelerometer data.
float angle_pitch, angle_roll, angle_yaw; // Combined angle (98% gyro + 2% accelerometer (for roll and pitch)).
float angle_pitch_constrained, angle_roll_constrained, angle_yaw_constrained; // Constrained combined angle.
bool set_gyro_angles; // Flag indicating whether the gyro-based angles have been initialized.

/*************************************************
 * Functions
 *************************************************/
void setup_mpu_6050_registers(); //Configures the MPU6050 registers.
bool i2cReadBytes(uint8_t addr, uint8_t startReg, uint8_t n, uint8_t* buf); //Reads 14 bytes on the I2C.
bool read_mpu_6050_data(); //Combines values from the 14 bytes into variables.
bool i2cRecoverBus(); //Try to recover a stuck I2C bus.
void i2cReinit(); //Reinitializes I2C bus.
void calibrate_mpu_6050(); 
void calc_angle(float dt);


void setup() {
  Wire.begin();
  Wire.setWireTimeout(3000, true); 
  Wire.setClock(100000); 
  Serial.begin(500000);
  setup_mpu_6050_registers();
  calibrate_mpu_6050();
}


void loop() {
  static uint32_t tPrev = micros();
  uint32_t tNow = micros();
  float dt = (tNow - tPrev) * 1e-6f;
  tPrev = tNow;
  
  if (dt < 0.002f || dt > 0.01f) dt = 0.004f;

  if (!read_mpu_6050_data()) {
  Serial.println("MPU read fail → Recovery");
  i2cReinit();
  setup_mpu_6050_registers();
  return;
  }
  
  calc_angle(dt);

  Serial.print("Roll: ");
  Serial.print(angle_roll_constrained, 2);
  Serial.print("Pitch: ");
  Serial.println(angle_pitch_constrained, 2);
}

/**
 * @brief Configures the MPU-6050 sensor registers via I2C.
 *
 * This function initializes and configures the MPU-6050 by writing
 * to its internal registers over the I2C bus.
 * 1. Wakes up the MPU-6050 by disabling sleep mode (PWR_MGMT_1).
 * 2. Configures the accelerometer full-scale range to ±8g.
 * 3. Configures the gyroscope full-scale range to ±500 degrees per second.
 * 4. Enables the Digital Low Pass Filter (DLPF) to reduce sensor noise
 *    with a cutoff frequency of approximately 42–44 Hz.
 */
void setup_mpu_6050_registers() {
  Wire.beginTransmission(0x68); //PWR_MGMT_1
  Wire.write(0x6B);
  Wire.write(0x00);
  Wire.endTransmission();

  Wire.beginTransmission(0x68); //ACC_CONFIG
  Wire.write(0x1C);
  Wire.write(0x10); 
  Wire.endTransmission();

  Wire.beginTransmission(0x68); //GYRO_CONFIG
  Wire.write(0x1B);
  Wire.write(0x08); 
  Wire.endTransmission();

  Wire.beginTransmission(0x68); //Low-pass filter
  Wire.write(0x1A);
  Wire.write(0x05);   // ~42 Hz gyro / 44 Hz accel
  Wire.endTransmission();
}

/**
 * @brief Reads a specific number of bytes from an I2C device with a safety timeout.
 * 
 * 1. Sends a 'Repeated Start' to the device to select the starting register.
 * 2. Requests 'n' bytes of data.
 * 3. Waits for data to arrive in the buffer, but aborts if it takes longer 
 *    than 3ms (3000 micros) to prevent the drone's software from freezing.
 * 4. Transfers the received bytes into the provided buffer.
 * 
 * @return true if all bytes were read successfully.
 * @return false on error or timeout.
 */
bool i2cReadBytes(uint8_t addr, uint8_t startReg, uint8_t n, uint8_t* buf) {
  Wire.beginTransmission(addr);
  Wire.write(startReg);
  if (Wire.endTransmission(false) != 0) return false;

  // Requests n bytes
  Wire.requestFrom((int)addr, (int)n, (int)true);
  unsigned long t0 = micros();
  while (Wire.available() < n) {
    if (micros() - t0 > 3000) { // 3 ms timeout
      return false;
    }
  }
  for (uint8_t i = 0; i < n; i++) 
  buf[i] = Wire.read();
  return true;
}

/**
 * @brief This function safely reads motion data from the sensor.
 *
 * 1. It checks if the sensor is responding. If not, it stops to avoid a crash.
 * 2. It collects an array of 14 bytes.
 * 3. Each variable is 2 bytes (2 8-bits). 
 * 4. It combines 2 bytes into a int16_t by type casting. Without int16_t, the data contained in it is unreadable
 *    For example acc_x contains 16 bits (2 8-bits) of information from the sensor. The sensor can only send 8-bits
 *    at a time. The first 8 bits is placed in position [0] and the second is placed in position [1] of the 14 bytes vector buf. 
 *
 * @return true if data is okay.
 * @return false if there was a connection error.
 */
bool read_mpu_6050_data() {
  uint8_t buf[14];
  if (!i2cReadBytes(0x68, 0x3B, 14, buf)) return false;

  acc_x = (int16_t)((buf[0] << 8) | buf[1]);
  acc_y = (int16_t)((buf[2] << 8) | buf[3]);
  acc_z = (int16_t)((buf[4] << 8) | buf[5]);
 // temperature = (int16_t)((buf[6] << 8) | buf[7]);
  gyro_x = (int16_t)((buf[8] << 8) | buf[9]);
  gyro_y = (int16_t)((buf[10] << 8) | buf[11]);
  gyro_z = (int16_t)((buf[12] << 8) | buf[13]);
  return true;
}

/**
 * @brief Attempts to recover a "hung" I2C bus.
 *
 * Sometimes a slave device (like the MPU6050) gets stuck in the middle of 
 * transmitting a byte and holds the SDA line LOW, preventing any further communication.
 * This function:
 * 1. Checks if the SDA line is stuck LOW.
 * 2. Manually toggles the SCL (clock) line up to 9 times to force the slave 
 *    to finish its current byte and release the SDA line.
 * 3. Sends a manual I2C STOP condition to formally reset the bus state.
 *
 * @return true when the recovery sequence is complete.
 */
bool i2cRecoverBus() {
  // Try to force SDA HIGH to sensor from holding the bus on LOW.
  pinMode(SCL, INPUT_PULLUP);
  pinMode(SDA, INPUT_PULLUP);
  delayMicroseconds(5);

  if (digitalRead(SDA) == LOW) {
    pinMode(SCL, OUTPUT);
    for (int i = 0; i < 9 && digitalRead(SDA) == LOW; i++) {
      digitalWrite(SCL, LOW);  delayMicroseconds(5);
      digitalWrite(SCL, HIGH); delayMicroseconds(5);
    }
    pinMode(SCL, INPUT_PULLUP);
  }
  // Send STOP to end the session.
  pinMode(SDA, OUTPUT); digitalWrite(SDA, LOW);  delayMicroseconds(5);
  pinMode(SCL, OUTPUT); digitalWrite(SCL, HIGH); delayMicroseconds(5);
  pinMode(SDA, INPUT_PULLUP);                   delayMicroseconds(5);
  pinMode(SCL, INPUT_PULLUP);
  return true;
}

/**
 * @brief Reinitializes the I2C interface in a safe and controlled way.
 *
 * 1. Stops the current Wire (I2C) session to reset the internal state machine.
 * 2. Calls i2cRecoverBus() to release the SDA/SCL lines if a slave device
 *    is holding the bus LOW.
 * 3. Reinitializes the Wire library.
 * 4. Reapplies timeout protection to avoid blocking behavior.
 * 5. Restores a conservative I2C clock speed for stable communication.
 */
void i2cReinit() {
  Wire.end();
  i2cRecoverBus();
  Wire.begin();
  Wire.setWireTimeout(3000, true);
  Wire.setClock(100000);
  delay(5);
}

void calibrate_mpu_6050() {
  gyro_x_cal = 0;
  gyro_y_cal = 0;
  gyro_z_cal = 0;

  for (int i = 0; i < 2000; i++) {
    if (!read_mpu_6050_data()) {
      Serial.println("MPU read fail → Recovery");
      i2cReinit();
      setup_mpu_6050_registers();
      i--; 
      delay(3);
      continue;
    }

    gyro_x_cal += gyro_x;
    gyro_y_cal += gyro_y;
    gyro_z_cal += gyro_z;

    delay(3);
  }

  gyro_x_cal /= 2000;
  gyro_y_cal /= 2000;
  gyro_z_cal /= 2000;
}

void calc_angle(float dt) {
  gyro_x -= gyro_x_cal;
  gyro_y -= gyro_y_cal;
  gyro_z -= gyro_z_cal;
  
  angle_pitch_gy += (gyro_x / 65.5f) * dt;
  angle_roll_gy  += (gyro_y / 65.5f) * dt;
  
  angle_pitch_acc = atan2(-(float)acc_x, sqrt((float)acc_y*(float)acc_y + (float)acc_z*(float)acc_z)) * 57.296f;
  angle_roll_acc  = atan2((float)acc_y, (float)acc_z) * 57.296f;

  if (set_gyro_angles) {
    angle_pitch = angle_pitch_gy * 0.98 + angle_pitch_acc * 0.02;
    angle_roll  = angle_roll_gy  * 0.98 + angle_roll_acc  * 0.02;
  } else {
    angle_pitch_gy = angle_pitch_acc;
    angle_roll_gy  = angle_roll_acc;
    angle_pitch = angle_pitch_acc;
    angle_roll  = angle_roll_acc;
    set_gyro_angles = true;
  }

  angle_roll_constrained = constrain(angle_roll, -90.0, 90.0);
  angle_pitch_constrained = constrain(angle_pitch, -90.0, 90.0);

}