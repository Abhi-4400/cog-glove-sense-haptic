#include <Wire.h>
#include <Adafruit_Sensor.h>
#include <Adafruit_BNO055.h>
#include <utility/imumaths.h>

/*----- IMU Setup -----*/
// Set the delay between fresh samples=
#define BNO055_SAMPLERATE_DELAY_MS (10) // 100 Hz

// Check I2C device address and correct line below (by default address is 0x29 or 0x28) id, address
Adafruit_BNO055 bno = Adafruit_BNO055(-1, 0x28, &Wire2);

struct __attribute__((__packed__)) IMUData
{
  // Quaternion
  float qw, qx, qy, qz;

  // Acceleration
  float ax, ay, az;
};

struct __attribute__((__packed__)) CalibrationData
{
  uint8_t sys, gyro, accel, mag;
};

/*----- Force Setup -----*/
// Define the analog pins based on the wiring table
const int forceThumb  = A0;
const int forceIndex  = A1;
const int forceMiddle = A12;
const int forceRing   = A13;
const int forcePinky  = A4;

const int numFingers = 5;

struct __attribute__((__packed__)) ForceData
{
  uint8_t fT, fI, fM, fR, fP;
};

// Serial communication bits
const byte START_BIT_IMU = 0xAA;
const byte START_BIT_FF = 0xBB;
const byte END_BIT = 0x55;

// State machine variables
uint8_t state = 0;
unsigned long imu_dt = 10000; // us, 100 Hz
unsigned long imu_prev_time = 0;
unsigned long ff_dt = 1000; // us, 1000 Hz
unsigned long ff_prev_time = 0;

void setup(void)
{
  Serial.begin(115200);

  while (!Serial) delay(10);  // wait for serial port to open!

  Wire2.begin();

  // Initialise BNO055
  if(!bno.begin())
  {
    Serial.print("BNO055 not detected");
    while(1);
  }

  delay(1000);

  bno.setExtCrystalUse(true);
}

void loop(void)
{
  switch (state) {
    case 0:
      {
        if (Serial.available() > 0)
        {
          int incomingByte = Serial.read();
          if (incomingByte == 1)
            state = 2; // IMU calibration status
        }
        else if (micros() - imu_prev_time >= imu_dt)
        {
          state = 1; // IMU quaternion and acceleration data
          imu_prev_time += imu_dt;
        }
        else if (micros() - ff_prev_time >= ff_dt)
        {
          state = 3; // Force data
          ff_prev_time += ff_dt;
        }
        break;
      }

    case 1: // Write IMU quaternion and acceleration data
      {
        imu::Quaternion quat = bno.getQuat();
        imu::Vector<3> accel = bno.getVector(Adafruit_BNO055::VECTOR_LINEARACCEL); // Filtered acceleration data
        IMUData imu_packet;

        imu_packet.qw = quat.w();
        imu_packet.qx = quat.x();
        imu_packet.qy = quat.y();
        imu_packet.qz = quat.z();
        
        imu_packet.ax = accel.x();
        imu_packet.ay = accel.y();
        imu_packet.az = accel.z();

        Serial.write(START_BIT_IMU); // start of message
        Serial.write((byte*)&imu_packet, sizeof(imu_packet)); // message
        Serial.write(END_BIT); // end of message
        
        state = 3;
        break;
      }
    case 2: // Write IMU calibration status
      {
        uint8_t system, gyroscope, accelerometer, magnetometer;
        bno.getCalibration(&system, &gyroscope, &accelerometer, &magnetometer);
        CalibrationData cal_packet;

        cal_packet.sys = system;
        cal_packet.gyro = gyroscope;
        cal_packet.accel = accelerometer;
        cal_packet.mag = magnetometer;

        Serial.write(START_BIT_IMU); // start of message
        Serial.write((byte*)&cal_packet, sizeof(cal_packet)); // message
        Serial.write(END_BIT); // end of message

        state = 0;
        break;
      }
    case 3: // Write force data
      {
        ForceData f_packet;

        f_packet.fT = analogRead(forceThumb);
        f_packet.fI = analogRead(forceIndex);
        f_packet.fM = analogRead(forceMiddle);
        f_packet.fR = analogRead(forceRing);
        f_packet.fP = analogRead(forcePinky);

        Serial.write(START_BIT_FF); // start of message
        Serial.write((byte*)&f_packet, sizeof(f_packet)); // message
        Serial.write(END_BIT); // end of message

        state = 0;
        break;
      }
  }
}