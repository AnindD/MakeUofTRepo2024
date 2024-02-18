byte count = 0;
String a;

#include "MPU9250.h"

MPU9250 mpu;

void setup() {
  Serial1.begin(115200);
  Serial.begin(115200);

  Wire.begin();
  delay(2000);


  if (!mpu.setup(0x68)) {  // change to your own address
      while (1) {
          Serial.println("MPU connection failed. Please check your connection with `connection_check` example.");
          delay(5000);
      }
  }
    // calibrate anytime you want to
  Serial.println("Accel Gyro calibration will start in 5sec.");
  Serial.println("Please leave the device still on the flat plane.");
  mpu.verbose(true);
  delay(5000);
  mpu.setAccBias(25.90, 21.51, 27.62); 
  mpu.setGyroBias(0.09,1.37,0.84); 
  mpu.setMagBias(0,0,0);
  mpu.setMagScale(0,0,0); 
  mpu.calibrateAccelGyro();


  //print_calibration();
  mpu.verbose(false);
}

void loop() {
  while (Serial1.available() > 0) {
    a += char(Serial1.read());
    delay(1);
  }
  if (a != "") {
    Serial.println(a);
    a = "";
  }
  if (mpu.update()) {
    static uint32_t prev_ms = millis();
    if (millis() > prev_ms + 25) {
        print_roll_pitch_yaw();
        prev_ms = millis();
  }
}
}

void print_roll_pitch_yaw() {
    Serial.print("Yaw, Pitch, Roll: ");
    Serial.print(mpu.getYaw(), 2);
    Serial.print(", ");
    Serial.print(mpu.getPitch(), 2);
    Serial.print(", ");
    Serial.println(mpu.getRoll(), 2);
}
