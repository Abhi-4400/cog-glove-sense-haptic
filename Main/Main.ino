#include <Wire.h>
#include <Adafruit_Sensor.h>
#include <Adafruit_BNO055.h>
#include <Adafruit_DRV2605.h>
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
  // Force
  uint16_t fT, fI, fM, fR, fP;

  // Flex
  uint16_t xT, xI, xM, xR, xP;
};

/*----- Haptic Setup -----*/
#define TCA9548A_ADDR 0x70
#define NUM_HAPTICS 5

const uint8_t HAPTIC_CHANNELS[NUM_HAPTICS] = {2, 3, 4, 5, 6};

Adafruit_DRV2605 drv; 

struct __attribute__((__packed__)) HapticCommand
{
  uint8_t driver;
  uint8_t effect; 
};

void tcaSelect(uint8_t channel)
{
  if (channel > 7) return;
  Wire1.beginTransmission(TCA9548A_ADDR);
  Wire1.write(1 << channel);
  Wire1.endTransmission();
}

bool receivingHaptic = false;
unsigned long haptic_wait_start = 0;
const unsigned long HAPTIC_WAIT_TIMEOUT_US = 5000;

/*----- Communication Setup -----*/
// Serial communication bits
const byte START_BIT = 0xAA;
const byte END_BIT = 0x55;

enum PacketType : uint8_t
{
  PACKET_IMU = 0,
  PACKET_FF = 1,
  PACKET_CALIB = 2,
  PACKET_HAPTIC = 3
};

enum PCCommand : uint8_t
{
  RETURN = 0,
  DEBUG = 1,
  CALIBRATION = 2,
  DATA_STREAM = 3
};

/*----- State Machine Setup -----*/
enum SystemState
{
  STATE_IDLE,
  STATE_DATA_STREAM,
  STATE_DEBUG
};

SystemState state = STATE_IDLE;
unsigned long imu_dt = 10000;  // us, 100 Hz
unsigned long imu_prev_time = 0;
unsigned long ff_dt = 5000;  // us, 200 Hz
unsigned long ff_prev_time = 0;
unsigned long debug_dt = 500000;  // us, 0.2 Hz
unsigned long debug_prev_time = 0;

void setup(void) {
  Serial.begin(115200);

  while (!Serial) delay(10);  // wait for serial port to open!

  Wire2.begin();
  Wire1.begin();

  // Initialise BNO055
  if (!bno.begin()) {
    Serial.print("BNO055 not detected");
    while (1);
  }

  delay(1000);

  bno.setExtCrystalUse(true);

  // Initialise Haptics
  for (uint8_t i = 0; i < NUM_HAPTICS; i++)
  {
    tcaSelect(HAPTIC_CHANNELS[i]);
    if (!drv.begin())
    {
      Serial.print("DRV2605 not detected on channel ");
      Serial.println(HAPTIC_CHANNELS[i]);
    }
    else
    {
      drv.selectLibrary(1);               
      drv.setMode(DRV2605_MODE_INTTRIG);  
    }
  }
}

void loop(void) {
  switch (state) {
    case STATE_IDLE:
      handleIdleState();
      break;

    case STATE_DATA_STREAM:
      handleSensorStreamState();
      break;

    case STATE_DEBUG:
      handleDebug();
      break;
  }
}

/*----- State Handler Functions -----*/
void handleIdleState()
{
  if (Serial.available() > 0) {
    int incomingByte = Serial.read();
    if (incomingByte == 0) {
      debug_prev_time = micros();

      state = STATE_DEBUG;
      return;
    }
    else if (incomingByte == 1) {
      CalibrationData imu_calibration_data;
      getCalibrationData(imu_calibration_data);
      sendData(imu_calibration_data, PACKET_CALIB);
      return;
    }
    else if (incomingByte == 2) {
      imu_prev_time = micros();
      ff_prev_time = micros();

      state = STATE_DATA_STREAM;
      return;
    }
  }
}

void handleSensorStreamState()
{
  unsigned long current_micros = micros();

  if (current_micros - imu_prev_time >= imu_dt) {
    IMUData imu_data;
    getIMUData(imu_data);
    sendData(imu_data, PACKET_IMU);
    imu_prev_time += imu_dt;
  }

  if (current_micros - ff_prev_time >= ff_dt) {
    ForceFlexData ff_data;
    getForceFlexData(ff_data);
    sendData(ff_data, PACKET_FF);
    ff_prev_time += ff_dt;
  }

  if (Serial.available() > 0) {
    // Peek at the byte first without consuming it
    int incoming_byte = Serial.peek(); 
    
    if (incoming_byte == 0) {
      Serial.read(); // Consume the byte
      state = STATE_IDLE;
    }
    else if (incoming_byte == 2) {
      // Only read if the full haptic payload has arrived in the buffer
      if (Serial.available() >= 1 + (int)sizeof(HapticCommand)) {
        Serial.read(); // Consume the command byte (2)
        
        HapticCommand cmd;
        Serial.readBytes((char*)&cmd, sizeof(cmd));

        startHaptics(cmd);
        sendData(cmd, PACKET_HAPTIC); // Corrected: Passed PACKET_HAPTIC
      }
    }
    else {
      // Clean up stray/unknown bytes so the buffer doesn't clog
      Serial.read();
    }
  }
}

void handleDebug()
{
  if (micros() - debug_prev_time >= debug_dt)
  {
    CalibrationData calib_data;
    getCalibrationData(calib_data);
    sendData(calib_data, PACKET_CALIB);

    IMUData imu_data;
    getIMUData(imu_data);
    sendData(imu_data, PACKET_IMU);
    
    ForceFlexData ff_data;
    getForceFlexData(ff_data);
    sendData(ff_data, PACKET_FF);

    debug_prev_time += debug_dt;
  }

  if (Serial.available() > 0) {
    if (Serial.read() == 0) {
      state = STATE_IDLE;
    }
  }
}

/*----- Send Function -----*/
template <typename T>
void sendData(const T& packet, PacketType type)
{
  Serial.write(START_BIT);                       // start of message
  Serial.write(type);                            // packet type
  Serial.write((byte*)&packet, sizeof(packet));  // message
  Serial.write(END_BIT);                         // end of message
}

/*----- Sensor Read Functions -----*/
void getIMUData(IMUData& imu_packet)
{
  imu::Quaternion quat = bno.getQuat();
  imu::Vector<3> accel = bno.getVector(Adafruit_BNO055::VECTOR_LINEARACCEL);  // Filtered acceleration data

  imu_packet.qw = quat.w();
  imu_packet.qx = quat.x();
  imu_packet.qy = quat.y();
  imu_packet.qz = quat.z();

  imu_packet.ax = accel.x();
  imu_packet.ay = accel.y();
  imu_packet.az = accel.z();
}

void getCalibrationData(CalibrationData& cal_packet)
{
  uint8_t system, gyroscope, accelerometer, magnetometer;
  bno.getCalibration(&system, &gyroscope, &accelerometer, &magnetometer);

  cal_packet.sys = system;
  cal_packet.gyro = gyroscope;
  cal_packet.accel = accelerometer;
  cal_packet.mag = magnetometer;
}

void getForceFlexData(ForceFlexData& ff_packet)
{
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
}

void startHaptics(HapticCommand& cmd)
{
  if (cmd.driver < NUM_HAPTICS)
  {
    tcaSelect(HAPTIC_CHANNELS[cmd.driver]);
    drv.setWaveform(0, cmd.effect); 
    drv.setWaveform(1, 0);          
    drv.go();
  }
}