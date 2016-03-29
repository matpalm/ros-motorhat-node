#include <ros/ros.h>
#include <sensor_msgs/Imu.h>
#include <std_msgs/Int16MultiArray.h>
#include <std_msgs/Int32.h>  // debug
#include <wiringPi.h>
#include <wiringPiI2C.h>

using namespace std;

// ------------------------------
// HAT control registers and data

const int I2C_ADDR = 0x60;

const int MODE1 = 0x00;
const int MODE2 = 0x01;
const int PRESCALE = 0xFE;
const int LED0_ON_L = 0x06;
const int LED0_ON_H = 0x07;
const int LED0_OFF_L = 0x08;
const int LED0_OFF_H = 0x09;
const int ALL_LED_ON_L = 0xFA;
const int ALL_LED_ON_H = 0xFB;
const int ALL_LED_OFF_L = 0xFC;
const int ALL_LED_OFF_H = 0xFD;

const int SLEEP = 0x10;
const int ALLCALL = 0x01;
const int OUTDRV = 0x04;

// --------------------------------------------------
// mappings from motorN to specific control registers
//                     m0  m1  m2  m3
const int pwmPin[] = {  8, 13,  2,  7 };
const int in1Pin[] = { 10, 11,  4,  5 };
const int in2Pin[] = {  9, 12,  3,  6 };

// -------------------------
// i2c and pwm setting utils

int fd = 0;
void i2c_setup() {
  fd = wiringPiI2CSetup(I2C_ADDR);
  if (fd == -1)
    cerr << "no i2c device found?" << endl;
}

void write(int reg, int data) {
  cout << ">write(" << reg << "," << data << ")" << endl;
  wiringPiI2CWriteReg8(fd, reg, data);
}

int read(int reg) {
  cout << ">read(" << reg << ")";
  int val = wiringPiI2CReadReg8(fd, reg);
  cout << " returning " << val << endl;
  return val;
}

void setPWM(int channel, int on, int off) {
  cout << ">setPWM(" << channel << "," << on << "," << off << ")" << endl;
  write(LED0_ON_L+4*channel, on & 0xFF);
  write(LED0_ON_H+4*channel, on >> 8);
  write(LED0_OFF_L+4*channel, off & 0xFF);
  write(LED0_OFF_H+4*channel, off >> 8);
}

void setPin(int pin, int val) {
  cout << ">setPin(" << pin << "," << val << ")" << endl;
  if (val == 0)
    setPWM(pin, 0, 4096);
  else if (val == 1)
    setPWM(pin, 4096, 0);
  else
    cerr << "invalid val for setPin(" << pin << "," << val << ")" << endl;
}

// --------
// user api

void init_hat() {
  // This is just cut and paste (including sleeps)
  // See original for some more explaination....
  cout << ">init" << endl;
  i2c_setup();
  write(ALL_LED_ON_L, 0);
  write(ALL_LED_ON_H, 0);
  write(ALL_LED_OFF_L, 0);
  write(ALL_LED_OFF_H, 0);
  write(MODE2, OUTDRV);
  write(MODE1, ALLCALL);
  usleep(5000);
  int mode1 = read(MODE1);
  cout << "mode1 = " << mode1 << endl;
  mode1 &= ~SLEEP;
  write(MODE1, mode1);
  usleep(5000);
  cout << "<init" << endl;
}

void setPWMFreq(int freq) {
  cout << ">setPWMFreq" << endl;
  float prescaleval = 25000000.0;
  prescaleval /= 4096.0;
  prescaleval /= freq;
  prescaleval -= 1.0;
  float prescale = floor(prescaleval + 0.5);
  int old_mode = read(MODE1);
  int new_mode = (old_mode & 0x7F) | 0x10;
  write(MODE1, new_mode);
  write(PRESCALE, int(floor(prescale)));
  write(MODE1, old_mode);
  usleep(5000);
  write(MODE1, old_mode | 0x80);
  cout << "<setPWMFreq" << endl;
}

void stop(int motor_id) {
  setPin(in1Pin[motor_id], 0);
  setPin(in2Pin[motor_id], 0);
}

void setSpeed(int motor_id, int speed) {
  if (motor_id < 0 || motor_id > 3) {
    cerr << "invalid motor_id " << motor_id;
    return;
  }
  if (speed < -255 || speed > 255) {
    cerr << "invalid speed " << speed;
    return;
  }

  if (speed == 0) {
    // RELEASE
    stop(motor_id);
  }
  else if (speed > 0) {
    // FORWARD
    setPin(in2Pin[motor_id], 0);
    setPin(in1Pin[motor_id], 1);
    setPWM(pwmPin[motor_id], 0, speed * 16);
  }
  else { // => speed < 0
    // BACKWARD
    setPin(in1Pin[motor_id], 0);
    setPin(in2Pin[motor_id], 1);
    setPWM(pwmPin[motor_id], 0, -speed * 16);
  }
}

void turnOffMotors() {
  cout << ">turnOffMotors" << endl;
  for (int i = 0; i < 4; ++i)
    stop(i);
}

// Receive a vector4 of int corresponding to the desired motor speeds.
//void cmdCallback(const std_msgs::Int16MultiArray::ConstPtr& msg) {
void cmdCallback(const std_msgs::Int32::ConstPtr& msg) {
  /*
  cout << msg->data[0] << " " << msg->data[1] << " " << msg->data[2]
       << " " << msg->data[3] << endl;
  for (int i = 0; i < 4; ++i)
    setSpeed(i, msg->data[i]);
  */
  cout << "CALLBACK " << msg->data << endl;
}

int main(int argc, char **argv) {
  // Setup.
  init_hat();
  setPWMFreq(1600);
  atexit(turnOffMotors);

  // Start ROS node stuff.
  cout << "s1" << endl;
  ros::init(argc, argv, "rover_motors");
  cout << "s2" << endl;  
  ros::NodeHandle node;
  cout << "s3" << endl;  
  node.subscribe("rcmd", 5, cmdCallback);
  cout << "s4" << endl;
  
  // Run until exit.
  ros::spin();
  return 0;
}


