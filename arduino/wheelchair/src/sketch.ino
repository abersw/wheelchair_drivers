#include <ros.h>
#include <ros/time.h>
#include <geometry_msgs/Twist.h>
#include <wheelchair_msgs/wheelVels.h>
//#include <std_msgs/String.h>
#include <std_msgs/Int16.h>
#include <std_msgs/Float32.h>
#include <math.h>
#include <SoftwareSerial.h>

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

std_msgs::Float32 lw_msg;
std_msgs::Float32 rw_msg;
ros::Publisher arduinolw("/arduino/lw", &lw_msg);
ros::Publisher arduinorw("/arduino/rw", &rw_msg);


// Declare functions
void setupPins();
void setupSerial();
void setupWiFi();
bool rosConnected();
//void onTwist(const geometry_msgs::Twist &msg);
void onTwist(const std_msgs::Float32 &lw, const std_msgs::Float32 &rw);
float mapPwm(float x, float out_min, float out_max);

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

  if ((lpwr == 128) && (rpwr == 128)) {
    digitalWrite(motorRelay, LOW);
  }
  else {
    digitalWrite(motorRelay, HIGH);
    analogWrite(leftMotor, lpwr);
    analogWrite(rightMotor, rpwr);
  }
  
  
}

ros::NodeHandle nh;
wheelchair_msgs::wheelVels motor_commands_msg;
//ros::Subscriber<geometry_msgs::Twist> sub("/wheelchair_robot/cmd_vel", &onTwist);
//ros::Subscriber<std_msgs::Float32> leftWheel("/lwheel_vtarget", &onTwist);
//ros::Subscriber<std_msgs::Float32> rightWheel("/rwheel_vtarget", &onTwist);
//ros::Subscriber<wheelchair_msgs::wheelVels> WheelVels("/motor_commands", &onTwist);
ros::Subscriber<wheelchair_msgs::wheelVels> rMotorCommands("/motor_commands", &rMotorCmdsCb);


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

