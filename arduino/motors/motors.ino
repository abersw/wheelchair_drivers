#include <ros.h>
#include <geometry_msgs/Twist.h>
#include <math.h>
#include <SoftwareSerial.h>
#include <Sabertooth.h>

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
  digitalWrite(lowerRelayPins[0], LOW);
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
	digitalWrite(lowerRelayPins[0], HIGH);
	for (int i = 0; i < 50; i++){
		Serial.println(i);
		ST.motor(1, (int)i);
  		ST.motor(2, (int)i);
  		delay(500);
	}
	digitalWrite(lowerRelayPins[0], HIGH);
	delay(5000);*/
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
	if (!rosConnected()) {
		haltRobot();
	}
	nh.spinOnce();
}


void haltRobot(){
  ST.stop(); //stop sabertooth connection
}