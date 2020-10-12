#include <ros.h>
#include <ros/time.h>
#include <geometry_msgs/Twist.h>
#include <wheelchair_msgs/wheelVels.h>
//#include <std_msgs/String.h>
#include <std_msgs/Bool.h>
#include <std_msgs/Int16.h>
#include <std_msgs/Float32.h>
#include <math.h>
#include <SoftwareSerial.h>
#include <sensor_msgs/Imu.h>
#include <sensor_msgs/MagneticField.h>
#include "GY_85.h"
#include <Wire.h>

/*
 * Hardware pins connected to the motor controller
*/
const byte leftMotor = 12; //signal cable to left channel of motor controller
const byte rightMotor = 13; //signal cable to right channel of motor controller
const byte motorRelay = 10; //relay to enable/disable the motor controller

//motor variables and speed limiters
const byte motorStopValue = 128;
const byte motorMaxValue = 76;
const byte motorMinValue = 180;

const byte SBT_MIN = 0;
const byte SBT_RANGE = 150;

GY_85 GY85;     //create the object

#define ACCEL_FACTOR                      0.000598550415   // (ADC_Value / Scale) * 9.80665            => Range : +- 2[g]
                                                           //                                             Scale : +- 16384
#define GYRO_FACTOR                       0.0010642        // (ADC_Value/Scale) * (pi/180)             => Range : +- 2000[deg/s]
                                                           //                                             Scale : +- 16.4[deg/s]

#define MAG_FACTOR                        15e-8

#define DEBUG_ACCEL 0
#define DEBUG_BAROM 0
#define DEBUG_MAGNO 0
#define DEBUG_GYROS 0


// Declare functions
void setupPins();
void setupSerial();
void setupWiFi();
bool rosConnected();
void getImudata();
//void onTwist(const geometry_msgs::Twist &msg);
void onTwist(const std_msgs::Float32 &lw, const std_msgs::Float32 &rw);
float mapPwm(float x, float out_min, float out_max);
void rMotorCmdsCb(const wheelchair_msgs::wheelVels& motor_commands_msg); //added this because of error

ros::NodeHandle nh;
wheelchair_msgs::wheelVels motor_commands_msg;
sensor_msgs::Imu  imu_msg;
//ros::Subscriber<geometry_msgs::Twist> sub("/wheelchair_robot/cmd_vel", &onTwist);
//ros::Subscriber<std_msgs::Float32> leftWheel("/lwheel_vtarget", &onTwist);
//ros::Subscriber<std_msgs::Float32> rightWheel("/rwheel_vtarget", &onTwist);
//ros::Subscriber<wheelchair_msgs::wheelVels> WheelVels("/motor_commands", &onTwist);
ros::Subscriber<wheelchair_msgs::wheelVels> rMotorCommands("/motor_commands", &rMotorCmdsCb);
ros::Subscriber<std_msgs::Bool> motorRelayEngage("/motor_relay", &motorRelayEngageCb);
std_msgs::Float32 lw_msg;
std_msgs::Float32 rw_msg;
ros::Publisher arduinolw("/arduino/lw", &lw_msg);
ros::Publisher arduinorw("/arduino/rw", &rw_msg);
ros::Publisher imuPub("/wheelchair_robot/imu_raw", &imu_msg);
char imu_link[] = "imu";

int ax;
int ay;
int az;

int cx;
int cy;
int cz;

float gx;
float gy;
float gz;
float gt;

bool _connected = false;
bool relayStatus;


void setup() {
	//here
	setupPins();
	setupSerial();
	nh.initNode();
  nh.advertise(arduinolw);
  nh.advertise(arduinorw);
  nh.subscribe(rMotorCommands);
  nh.advertise(imuPub);
  Wire.begin();
  //delay(10);
  GY85.init();
  //delay(10);
	//nh.subscribe(leftWheel);
  //nh.subscribe(rightWheel);
	//haltRobot();
  stopBothMotors();
	//digitalWrite(lowerRelayPins[0], LOW);
  //delay(1000);
}

void loop() {
	//here too
	/*if (!rosConnected()) {
		haltRobot();
	}*/
    //digitalWrite(lowerRelayPins[0], LOW);
  //ST.motor(1, (int)40);
    getImuData();
  	nh.spinOnce();
}

void setupPins() {
	pinMode(leftMotor, OUTPUT); //set left motor as output for sending PWM signals
  pinMode(rightMotor, OUTPUT); //set right motor as output for sending PWM signals
  pinMode(motorRelay, OUTPUT); //set motor controller relay as output for sending High and Low signals
  stopBothMotors(); //set motors to stop on startup
}

void setupSerial() {
  
  stopBothMotors();
	Serial.begin(115200);
}

void motorRelayEngageCb(const std_msgs::Bool& motor_relay_msg) {
  if (motor_relay_msg.data == true) {
    analogWrite(leftMotor, motorStopValue);
    analogWrite(rightMotor, motorStopValue);
    digitalWrite(motorRelay, HIGH); //relay on
  }
  else {
    digitalWrite(motorRelay, LOW); //relay off
    analogWrite(leftMotor, motorStopValue);
    analogWrite(rightMotor, motorStopValue);
  }
}

void rMotorCmdsCb(const wheelchair_msgs::wheelVels& motor_commands_msg){
  float x = max(min(motor_commands_msg.leftVel, 1.0f), -1.0f);
  float z = max(min(motor_commands_msg.rightVel, 1.0f), -1.0f);

  // Calculate the intensity of left and right wheels. Simple version.
  // Taken from https://hackernoon.com/unicycle-to-differential-drive-courseras-control-of-mobile-robots-with-ros-and-rosbots-part-2-6d27d15f2010#1e59
  //float l = (msg.linear.x - msg.angular.z) / 2; //switches direction modes
  //float r = (msg.linear.x + msg.angular.z) / 2; //what about dynamic 127:0:127?

  // Then map those values to PWM intensities. PWMRANGE = full speed, while PWM_MIN = the minimal amount of power at which the motors begin moving.


  uint16_t lpwr = mapPwr(x, motorStopValue, motorMaxValue);
  uint16_t rpwr = mapPwr(z, motorStopValue, motorMaxValue);
  //float lpwr = map(lw, 0, 1.22, 0, 100);
  //uint16_t rpwr = map(fabs(r), 0, 1.22, 0, 100);

  
  //ST.motor(1, (int)lpwr);
  //ST.motor(2, (int)rpwr);
  
  //ST.motor(1, (int)40);

  lw_msg.data = lpwr;
  rw_msg.data = rpwr;
  arduinolw.publish( &lw_msg );
  arduinorw.publish( &rw_msg);

  analogWrite(leftMotor, lpwr);
  analogWrite(rightMotor, rpwr);
  
  
}

void getImuData() {
  
  imu_msg.header.frame_id = imu_link;
  imu_msg.header.stamp = nh.now();

  //we are not providing orientation, so set to -1
  //imu_msg.orientation_covariance[0] = -1;
  imu_msg.orientation.w = -1;
  imu_msg.orientation.x = -1;
  imu_msg.orientation.y = -1;
  imu_msg.orientation.z = -1;

  imu_msg.orientation_covariance[0] = -1;
  imu_msg.orientation_covariance[1] = -1;
  imu_msg.orientation_covariance[2] = -1;
  imu_msg.orientation_covariance[3] = -1;
  imu_msg.orientation_covariance[4] = -1;
  imu_msg.orientation_covariance[5] = -1;
  imu_msg.orientation_covariance[6] = -1;
  imu_msg.orientation_covariance[7] = -1;
  imu_msg.orientation_covariance[8] = -1;


  ax = GY85.accelerometer_x( GY85.readFromAccelerometer() );
  ay = GY85.accelerometer_y( GY85.readFromAccelerometer() );
  az = GY85.accelerometer_z( GY85.readFromAccelerometer() );

  cx = GY85.compass_x( GY85.readFromCompass() );
  cy = GY85.compass_y( GY85.readFromCompass() );
  cz = GY85.compass_z( GY85.readFromCompass() );

  gx = GY85.gyro_x( GY85.readGyro() );
  gy = GY85.gyro_y( GY85.readGyro() );
  gz = GY85.gyro_z( GY85.readGyro() );
  gt = GY85.temp  ( GY85.readGyro() );


  imu_msg.angular_velocity.x = gx * DEG_TO_RAD; //imu is mounted at 90 degrees, may need to +/- depending on axis
  imu_msg.angular_velocity.y = gy * DEG_TO_RAD; //imu is mounted at 90 degrees, may need to +/- depending on axis
  imu_msg.angular_velocity.z = gz * DEG_TO_RAD; //imu is mounted upsidedown, may need to +/- depending on axis

  //angular velocity covariance
  /*imu_msg.angular_velocity_covariance[0] = 0.003;
  imu_msg.angular_velocity_covariance[4] = 0.003;
  imu_msg.angular_velocity_covariance[8] = 0.003;
  */
  imu_msg.angular_velocity_covariance[0] = 0.02;
  imu_msg.angular_velocity_covariance[1] = 0;
  imu_msg.angular_velocity_covariance[2] = 0;
  imu_msg.angular_velocity_covariance[3] = 0;
  imu_msg.angular_velocity_covariance[4] = 0.02;
  imu_msg.angular_velocity_covariance[5] = 0;
  imu_msg.angular_velocity_covariance[6] = 0;
  imu_msg.angular_velocity_covariance[7] = 0;
  imu_msg.angular_velocity_covariance[8] = 0.02;

  imu_msg.linear_acceleration.x = ax; //imu is mounted at 90 degrees, may need to +/- depending on axis
  imu_msg.linear_acceleration.y = ay; //imu is mounted at 90 degrees, may need to +/- depending on axis
  imu_msg.linear_acceleration.z = az; //imu is mounted at 90 degrees, may need to +/- depending on axis

  /*imu_msg.linear_acceleration_covariance[0] = 0.1;
  imu_msg.linear_acceleration_covariance[4] = 0.1;
  imu_msg.linear_acceleration_covariance[8] = 0.1;
*/

  imu_msg.linear_acceleration_covariance[0] = 0.04;
  imu_msg.linear_acceleration_covariance[1] = 0;
  imu_msg.linear_acceleration_covariance[2] = 0;
  imu_msg.linear_acceleration_covariance[3] = 0;
  imu_msg.linear_acceleration_covariance[4] = 0.04;
  imu_msg.linear_acceleration_covariance[5] = 0;
  imu_msg.linear_acceleration_covariance[6] = 0;
  imu_msg.linear_acceleration_covariance[7] = 0;
  imu_msg.linear_acceleration_covariance[8] = 0.04;
  
  /*Serial.print  ( "accelerometer" );
    Serial.print  ( " x:" );
    Serial.print  ( ax );
    Serial.print  ( " y:" );
    Serial.print  ( ay );
    Serial.print  ( " z:" );
    Serial.print  ( az );

    Serial.print  ( "  compass" );
    Serial.print  ( " x:" );
    Serial.print  ( cx );
    Serial.print  ( " y:" );
    Serial.print  ( cy );
    Serial.print  (" z:");
    Serial.print  ( cz );

    Serial.print  ( "  gyro" );
    Serial.print  ( " x:" );
    Serial.print  ( gx );
    Serial.print  ( " y:" );
    Serial.print  ( gy );
    Serial.print  ( " z:" );
    Serial.print  ( gz );
    Serial.print  ( " gyro temp:" );
    Serial.println( gt );
  */

  //delay(100);             // only read every 0,5 seconds, 10ms for 100Hz, 20ms for 50Hz
  imuPub.publish(&imu_msg);
}


bool rosConnected() {
	bool connected = nh.connected();
	if (_connected != connected) {
		_connected = connected;
		digitalWrite(LED_BUILTIN, !connected);
		Serial.println(connected ? "ROS connected" : "ROS disconnected");
	}
	return connected;
}

float mapPwr(float x, float out_min, float out_max) {
	return x * (out_max - out_min) + out_min;
}



/*
 * Function to stop both motors using stopping speed value
*/
void stopBothMotors() {
  analogWrite(leftMotor, motorStopValue);
  analogWrite(rightMotor, motorStopValue);
  digitalWrite(motorRelay, LOW);
  //Serial.println(commandValue);
}



/*
ros::NodeHandle nh;
bool _connected = false;

const byte loopcount = 1;

float mapPwm(float x, float out_min, float out_max);

// Define Lower Relay Pins
// Motor Relay-22, Unused-23
// Infrared Relay-24, IMU/Fall Sensor Relay-25
const byte lowerRelayPins[] = {22, 23, 24, 25};


// Define Upper Relay Pins
// Slam Device Relay-26, Upper Lights Relay-27
// Lower Lights Relay-28, Unused-29
const byte upperRelayPins[] = {26, 27, 28, 29};

// Define Motor Serial Tx Pin
const byte motorPin = 10;

const byte minival = -127;
const byte maxival = 127;

const byte arcRobot = true; //var to dictate this is the arc model robot

SoftwareSerial SWSerial(NOT_A_PIN, motorPin);
Sabertooth ST(128,SWSerial);

void relayOnOff(byte relayNum, byte on=0){
  switch (relayNum){
    case 1:
      //nh.logdebug("Changing state of motor controller power relay");
      if (on == 1){
        digitalWrite(lowerRelayPins[0], HIGH);
      } else if (on == 0){
        digitalWrite(lowerRelayPins[0], LOW);
      }
      break;
    case 2:
      //nh.logdebug("Changing state of unused power relay");
      if (on == 1){
        digitalWrite(lowerRelayPins[1], HIGH);
      } else if (on == 0){
        digitalWrite(lowerRelayPins[1], LOW);
      }
      break;
    case 3:
      //nh.logdebug("Changing state of infrared sensors power relay");
      if (on == 1){
        digitalWrite(lowerRelayPins[2], HIGH);
      } else if (on == 0){
        digitalWrite(lowerRelayPins[2], LOW);
      }
      break;
    case 4:
      //nh.logdebug("Changing state of IMU and fall sensors power relay");
      if (on == 1){
        digitalWrite(lowerRelayPins[3], HIGH);
      } else if (on == 0){
        digitalWrite(lowerRelayPins[3], LOW);
      }
      break;
    case 5:
      //nh.logdebug("Changing state of slam device power relay");
      if (on == 1){
        digitalWrite(upperRelayPins[0], HIGH);
      } else if (on == 0){
        digitalWrite(upperRelayPins[0], LOW);
      }
      break;
  }
}

bool rosConnected()
{
  // If value changes, notify via LED and console.
  bool connected = nh.connected();
  if (_connected != connected)
  {
    _connected = connected;
    digitalWrite(LED_BUILTIN, !connected); // false -> on, true -> off
    Serial.println(connected ? "ROS connected" : "ROS disconnected");
  }
  return connected;
}

//function from github - diff_drive magician chassis
void onTwist(const geometry_msgs::Twist &msg)
{
	
  if (!_connected)
  {
    haltRobot();
    return;
  }

  // Cap values at [-1 .. 1]
  float x = max(min(msg.linear.x, 1.0f), -1.0f);
  float z = max(min(msg.angular.z, 1.0f), -1.0f);

  // Calculate the intensity of left and right wheels. Simple version.
  // Taken from https://hackernoon.com/unicycle-to-differential-drive-courseras-control-of-mobile-robots-with-ros-and-rosbots-part-2-6d27d15f2010#1e59
  float l = (msg.linear.x - msg.angular.z) / 2;
  float r = (msg.linear.x + msg.angular.z) / 2;

  // Then map those values to PWM intensities. PWMRANGE = full speed, while PWM_MIN = the minimal amount of power at which the motors begin moving.
  /*uint16_t lPwm = mapPwm(fabs(l), PWM_MIN, PWMRANGE);
  uint16_t rPwm = mapPwm(fabs(r), PWM_MIN, PWMRANGE);

  // Set direction pins and PWM
  digitalWrite(L_FORW, l > 0);
  digitalWrite(L_BACK, l < 0);
  digitalWrite(R_FORW, r > 0);
  digitalWrite(R_BACK, r < 0);
  analogWrite(L_PWM, lPwm);
  analogWrite(R_PWM, rPwm);*/
  /*digitalWrite(lowerRelayPins[0], LOW);
  ST.motor(1, (int)l);
  ST.motor(2, (int)r);
  
}


ros::Subscriber<geometry_msgs::Twist> sub("/cmd_vel", &onTwist);

void setupMotorController(){
  //nh.loginfo("Initialising motor controller");
  //if (arcRobot == false){
  // relayOnOff(1,1);
  //} else if (arcRobot == true){
  // relayOnOff(1,0);
  //}
  //delay(2000);
nh.initNode();
  SWSerial.begin(38400);
  haltRobot();
  nh.loginfo("Motor controller initialisation complete");
}

void setup() {
	setupPins(); //setup hardware
	setupMotorController(); //setup motor controller
	setupSerial(); //setup serial connections

	/*
	if (!rosConnected()) {
		haltRobot();
	}
	nh.spinOnce();
	*/
	/*
	Serial.println("inside loop");
	digitalWrite(lowerRelayPins[0], LOW);
	for (int i = 0; i < 50; i++){
		Serial.println(i);
		ST.motor(1, (int)i);
  		ST.motor(2, (int)i);
  		delay(500);
	}
	digitalWrite(lowerRelayPins[0], HIGH);
	delay(5000);*//*
}

void setupPins() {
	for (int i = 0; i < 4; i++) {
		pinMode(lowerRelayPins[i], OUTPUT);
		digitalWrite(lowerRelayPins[i], HIGH); //high is low!!
	}
}

void setupSerial() {
	Serial.begin(115200); //give access to usb
}





void loop() {
	/*if (!rosConnected()) {
		haltRobot();
	}
	nh.spinOnce();*//*
}


void haltRobot(){
  ST.stop(); //stop sabertooth connection
}*/
