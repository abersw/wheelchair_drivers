#include <ros.h>
#include <ros/time.h>
#include <geometry_msgs/Twist.h>
#include <wheelchair_msgs/wheelVels.h>
//#include <std_msgs/String.h>
#include <std_msgs/Int16.h>
#include <std_msgs/Float32.h>
#include <math.h>
#include <SoftwareSerial.h>
#include <Sabertooth.h>

// Define Motor Serial Tx Pin
const byte motorPin = 10;

// Define Lower Relay Pins
// Motor Relay-22, Unused-23
// Infrared Relay-24, IMU/Fall Sensor Relay-25
const byte lowerRelayPins[] = {22, 23, 24, 25};


// Define Upper Relay Pins
// Slam Device Relay-26, Upper Lights Relay-27
// Lower Lights Relay-28, Unused-29
const byte upperRelayPins[] = {26, 27, 28, 29};

const byte SBT_MIN = 0;
const byte SBT_RANGE = 150;

SoftwareSerial SWSerial(NOT_A_PIN, motorPin);
Sabertooth ST(128,SWSerial);


// Declare functions
void setupPins();
void setupSerial();
void setupWiFi();
void rMotorCmdsCb(const wheelchair_msgs::wheelVels &whlmsgs);
void haltRobot();
float mapPwr(float x, float y, float z);
//void onTwist(const geometry_msgs::Twist &msg);
void onTwist(const std_msgs::Float32 &lw, const std_msgs::Float32 &rw);
float mapPwm(float x, float out_min, float out_max);

ros::NodeHandle nh;
wheelchair_msgs::wheelVels motor_commands_msg;
//ros::Subscriber<geometry_msgs::Twist> sub("/wheelchair_robot/cmd_vel", &onTwist);
//ros::Subscriber<std_msgs::Float32> leftWheel("/lwheel_vtarget", &onTwist);
//ros::Subscriber<std_msgs::Float32> rightWheel("/rwheel_vtarget", &onTwist);
//ros::Subscriber<wheelchair_msgs::wheelVels> WheelVels("/motor_commands", &onTwist);
ros::Subscriber<wheelchair_msgs::wheelVels> rMotorCommands("/motor_commands", &rMotorCmdsCb);
std_msgs::Float32 lw_msg;
std_msgs::Float32 rw_msg;
ros::Publisher arduinolw("/arduino/lw", &lw_msg);
ros::Publisher arduinorw("/arduino/rw", &rw_msg);

bool _connected = false;


void setup() {
	//here
	setupPins();
	setupSerial();
	nh.initNode();
  nh.advertise(arduinolw);
  nh.advertise(arduinorw);
  nh.subscribe(rMotorCommands);
	//nh.subscribe(leftWheel);
  //nh.subscribe(rightWheel);
	haltRobot();
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
  	nh.spinOnce();
}

void setupPins() {
	pinMode(LED_BUILTIN, OUTPUT);
	for (int i = 0; i < 4; i++) {
		pinMode(lowerRelayPins[i], OUTPUT);
		digitalWrite(lowerRelayPins[i], HIGH); //high is low!!
	}
}

void setupSerial() {
  SWSerial.begin(38400);
  haltRobot();
	Serial.begin(115200);
}

void rMotorCmdsCb(const wheelchair_msgs::wheelVels& motor_commands_msg){
  float x = max(min(motor_commands_msg.leftVel, 1.0f), -1.0f);
  float z = max(min(motor_commands_msg.rightVel, 1.0f), -1.0f);

  // Calculate the intensity of left and right wheels. Simple version.
  // Taken from https://hackernoon.com/unicycle-to-differential-drive-courseras-control-of-mobile-robots-with-ros-and-rosbots-part-2-6d27d15f2010#1e59
  //float l = (msg.linear.x - msg.angular.z) / 2; //switches direction modes
  //float r = (msg.linear.x + msg.angular.z) / 2; //what about dynamic 127:0:127?

  // Then map those values to PWM intensities. PWMRANGE = full speed, while PWM_MIN = the minimal amount of power at which the motors begin moving.


  uint16_t lpwr = mapPwr(x, SBT_MIN, SBT_RANGE);
  uint16_t rpwr = mapPwr(z, SBT_MIN, SBT_RANGE);
  //float lpwr = map(lw, 0, 1.22, 0, 100);
  //uint16_t rpwr = map(fabs(r), 0, 1.22, 0, 100);

  digitalWrite(lowerRelayPins[0], LOW);
  //ST.motor(1, (int)lpwr);
  //ST.motor(2, (int)rpwr);
  
  //ST.motor(1, (int)40);

  lw_msg.data = lpwr;
  rw_msg.data = rpwr;
  arduinolw.publish( &lw_msg );
  arduinorw.publish( &rw_msg);

  ST.motor(1, (int)lpwr);
  ST.motor(2, (int)rpwr);
}




float mapPwr(float x, float out_min, float out_max) {
	return x * (out_max - out_min) + out_min;
}

void haltRobot(){
	digitalWrite(lowerRelayPins[0], HIGH);
	ST.stop(); //stop sabertooth connection
}
