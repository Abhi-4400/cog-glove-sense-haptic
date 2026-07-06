#include <Wire.h>
#include <Adafruit_Sensor.h>
#include <Adafruit_BNO055.h>
#include <utility/imumaths.h>

/*----- IMU Setup -----*/
// Set the delay between fresh samples=
#define BNO055_SAMPLERATE_DELAY_MS (10)  // 100 Hz

// Check I2C device address and correct line below (by default address is 0x29 or 0x28) id, address
Adafruit_BNO055 bno = Adafruit_BNO055(-1, 0x28, &Wire2);

struct __attribute__((__packed__)) IMUData {
  // Quaternion
  float qw, qx, qy, qz;

  // Acceleration
  float ax, ay, az;
};

struct __attribute__((__packed__)) CalibrationData {
  uint8_t sys, gyro, accel, mag;
};

/*----- Force Flex Setup -----*/
// Define force pins
const int forceThumb = A0;
const int forceIndex = A1;
const int forceMiddle = A12;
const int forceRing = A13;
const int forcePinky = A4;

// Define flex pins
const int flexThumb = A5;
const int flexIndex = A6;
const int flexMiddle = A7;
const int flexRing = A8;
const int flexPinky = A9;

struct __attribute__((__packed__)) ForceFlexData {
  uint16_t fT, fI, fM, fR, fP;
  uint16_t xT, xI, xM, xR, xP;
};

/*----- Communication Setup -----*/
// Serial communication bits
const byte START_BIT_IMU = 0xAA;
const byte START_BIT_CALIB = 0xBB;
const byte START_BIT_FF = 0xCC;
const byte END_BIT = 0x55;

/*----- State Machine Setup -----*/
enum SystemState
{
  STATE_IDLE,
  STATE_SEND_IMU,
  STATE_SEND_CALIB,
  STATE_SEND_FORCE_FLEX
};

uint8_t state = STATE_IDLE;
unsigned long imu_dt = 10000;  // us, 100 Hz
unsigned long imu_prev_time = 0;
unsigned long ff_dt = 1000;  // us, 1000 Hz
unsigned long ff_prev_time = 0;

void setup(void) {
  Serial.begin(115200);

  while (!Serial) delay(10);  // wait for serial port to open!

  Wire2.begin();

  // Initialise BNO055
  if (!bno.begin()) {
    Serial.print("BNO055 not detected");
    while (1);
  }

  delay(1000);

  bno.setExtCrystalUse(true);
}

void loop(void) {
  switch (state) {
    case STATE_IDLE:
      handleIdleState();
      break;

    case STATE_SEND_IMU:  // Write IMU quaternion and acceleration data
      sendIMUData();
      break;

    case STATE_SEND_CALIB:  // Write IMU calibration status
      sendCalibrationData();
      break;

    case STATE_SEND_FORCE_FLEX:  // Write force and flex data
      sendForceFlexData();
      break;
  }
}

/*----- State Handler Functions -----*/
void handleIdleState()
{
  // 1. Check Serial input independently (highest priority command)
  if (Serial.available() > 0) {
    int incomingByte = Serial.read();
    if (incomingByte == 1) {
      state = STATE_SEND_CALIB;
      return; // Interrupt everything to send calibration
    }
  } 
  
  unsigned long currentMicros = micros();

  // 2. Evaluate timing events
  // If both are ready, IMU triggers first, but Force/Flex will trigger immediately on the very next loop cycle.
  if (currentMicros - imu_prev_time >= imu_dt) {
    state = STATE_SEND_IMU;
    imu_prev_time += imu_dt;
    return;
  }

  if (currentMicros - ff_prev_time >= ff_dt) {
    state = STATE_SEND_FORCE_FLEX;
    ff_prev_time += ff_dt;
    return;
  }
}

void sendIMUData()
{
  imu::Quaternion quat = bno.getQuat();
  imu::Vector<3> accel = bno.getVector(Adafruit_BNO055::VECTOR_LINEARACCEL);  // Filtered acceleration data
  IMUData imu_packet;

  imu_packet.qw = quat.w();
  imu_packet.qx = quat.x();
  imu_packet.qy = quat.y();
  imu_packet.qz = quat.z();

  imu_packet.ax = accel.x();
  imu_packet.ay = accel.y();
  imu_packet.az = accel.z();

  Serial.write(START_BIT_IMU);                           // start of message
  Serial.write((byte*)&imu_packet, sizeof(imu_packet));  // message
  Serial.write(END_BIT);                                 // end of message

  state = STATE_IDLE;
}

void sendCalibrationData()
{
  uint8_t system, gyroscope, accelerometer, magnetometer;
  bno.getCalibration(&system, &gyroscope, &accelerometer, &magnetometer);
  CalibrationData cal_packet;

  cal_packet.sys = system;
  cal_packet.gyro = gyroscope;
  cal_packet.accel = accelerometer;
  cal_packet.mag = magnetometer;

  Serial.write(START_BIT_CALIB);                           // start of message
  Serial.write((byte*)&cal_packet, sizeof(cal_packet));  // message
  Serial.write(END_BIT);                                 // end of message

  state = STATE_IDLE;
}

void sendForceFlexData()
{
  ForceFlexData ff_packet;

  ff_packet.fT = analogRead(forceThumb);
  ff_packet.fI = analogRead(forceIndex);
  ff_packet.fM = analogRead(forceMiddle);
  ff_packet.fR = analogRead(forceRing);
  ff_packet.fP = analogRead(forcePinky);

  ff_packet.xT = analogRead(flexThumb);
  ff_packet.xI = analogRead(flexIndex);
  ff_packet.xM = analogRead(flexMiddle);
  ff_packet.xR = analogRead(flexRing);
  ff_packet.xP = analogRead(flexPinky);

  Serial.write(START_BIT_FF);                          // start of message
  Serial.write((byte*)&ff_packet, sizeof(ff_packet));  // message
  Serial.write(END_BIT);                               // end of message

  state = STATE_IDLE;
}