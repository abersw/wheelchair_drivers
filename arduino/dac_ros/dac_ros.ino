#include <ros.h>
#include <ros/time.h>
#include <geometry_msgs/Twist.h>
#include <wheelchair_msgs/wheelVels.h>
//#include <std_msgs/String.h>
#include <std_msgs/Int16.h>
#include <std_msgs/Float32.h>
#include <math.h>
#include <SoftwareSerial.h>
#include <Adafruit_MCP4725.h>
#define FB_voltsIn 20 //SDA on nano = A4 | SDA on Mega = 20
#define RL_voltsIn 20

/*
   Hardware pins connected to the motor controller
*/

Adafruit_MCP4725 FB_dac; // constructor
Adafruit_MCP4725 RL_dac;



// Declare functions
void setupPins();
void setupSerial();
void setupWiFi();
bool rosConnected();
//void onTwist(const geometry_msgs::Twist &msg);
void onTwist(const std_msgs::Float32 &lw, const std_msgs::Float32 &rw);
float mapPwm(float x, float out_min, float out_max);

ros::NodeHandle nh;
//wheelchair_msgs::wheelVels motor_commands_msg;
ros::Subscriber<geometry_msgs::Twist> sub("/wheelchair_robot/cmd_vel", &rMotorCmdsCb);
//ros::Subscriber<std_msgs::Float32> leftWheel("/lwheel_vtarget", &onTwist);
//ros::Subscriber<std_msgs::Float32> rightWheel("/rwheel_vtarget", &onTwist);
//ros::Subscriber<wheelchair_msgs::wheelVels> WheelVels("/motor_commands", &onTwist);
//ros::Subscriber<wheelchair_msgs::wheelVels> rMotorCommands("/motor_commands", &rMotorCmdsCb);
std_msgs::Float32 lw_msg;
std_msgs::Float32 rw_msg;
ros::Publisher arduinolw("/arduino/lw", &lw_msg);
ros::Publisher arduinorw("/arduino/rw", &rw_msg);

bool _connected = false;


void setup() {
  //here
  FB_dac.begin(0x60); // The I2C Address: Run the I2C Scanner if you're not sure
  RL_dac.begin(0x61);
  setupPins();
  setupSerial();
  nh.initNode();
  nh.advertise(arduinolw);
  nh.advertise(arduinorw);
  nh.subscribe(sub);
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
  /*  pinMode(leftMotor, OUTPUT); //set left motor as output for sending PWM signals
    pinMode(rightMotor, OUTPUT); //set right motor as output for sending PWM signals
    pinMode(motorRelay, OUTPUT); //set motor controller relay as output for sending High and Low signals
    stopBothMotors(); //set motors to stop on startup*/
}

void setupSerial() {

  stopBothMotors();
  Serial.begin(115200);
}

void rMotorCmdsCb(const geometry_msgs::Twist &msg) {
  //float x = max(min(motor_commands_msg.leftVel, 1.0f), -1.0f);
  //float z = max(min(motor_commands_msg.rightVel, 1.0f), -1.0f);
  // Cap values at [-1 .. 1]
  float x = max(min(msg.linear.x, 1.0f), -1.0f);
  float z = max(min(msg.angular.z, 1.0f), -1.0f);

  // Calculate the intensity of left and right wheels. Simple version.
  // Taken from https://hackernoon.com/unicycle-to-differential-drive-courseras-control-of-mobile-robots-with-ros-and-rosbots-part-2-6d27d15f2010#1e59
  //float l = (msg.linear.x - msg.angular.z) / 2;
  //float r = (msg.linear.x + msg.angular.z) / 2;

  //max lienar velocity from teleop is 1.22
  //min linear velocity from teleop is -1.22
  //max angular velocity from teleop is 2.84
  //min angular velocity from teleop is -2.84

  uint32_t FB_dac_value = x;
  //uint32_t FB_dac_value = 2250;
  int FB_adcValueRead = 0;
  float FB_voltageRead = 0;
  float FB_dac_expected_output;

  FB_dac_expected_output = (5.0 / 4096.0) * FB_dac_value;
  FB_dac.setVoltage(FB_dac_value, false);
  delay(250);
  FB_adcValueRead = analogRead(FB_voltsIn);
  FB_voltageRead = (FB_adcValueRead * 5.0 ) / 1024.0;

  uint32_t RL_dac_value = z;
  //uint32_t RL_dac_value = 2250;
  int RL_adcValueRead = 0;
  float RL_voltageRead = 0;
  float RL_dac_expected_output;

  RL_dac_expected_output = (5.0 / 4096.0) * FB_dac_value;
  RL_dac.setVoltage(RL_dac_value, false);
  delay(250);
  RL_adcValueRead = analogRead(RL_voltsIn);
  RL_voltageRead = (RL_adcValueRead * 5.0 ) / 1024.0;

  lw_msg.data = FB_voltageRead;
  rw_msg.data = RL_voltageRead;
  arduinolw.publish( &lw_msg );
  arduinorw.publish( &rw_msg);

  // Calculate the intensity of left and right wheels. Simple version.
  // Taken from https://hackernoon.com/unicycle-to-differential-drive-courseras-control-of-mobile-robots-with-ros-and-rosbots-part-2-6d27d15f2010#1e59
  //float l = (msg.linear.x - msg.angular.z) / 2; //switches direction modes
  //float r = (msg.linear.x + msg.angular.z) / 2; //what about dynamic 127:0:127?

  // Then map those values to PWM intensities. PWMRANGE = full speed, while PWM_MIN = the minimal amount of power at which the motors begin moving.


  /*  uint16_t lpwr = mapPwr(x, motorStopValue, motorMaxValue);
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
    else {*/
  /*digitalWrite(motorRelay, HIGH);
    analogWrite(leftMotor, lpwr);
    analogWrite(rightMotor, rpwr);*/
  //}


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
   Function to stop both motors using stopping speed value
*/
void stopBothMotors() {
  /*  analogWrite(leftMotor, motorStopValue);
    analogWrite(rightMotor, motorStopValue);
    digitalWrite(motorRelay, LOW);*/
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
