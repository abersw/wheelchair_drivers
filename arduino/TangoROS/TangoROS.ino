#include <ros.h>
#include <ros/time.h>
#include <std_msgs/Int8.h>
#include <std_msgs/Int16.h>
#include <std_msgs/Float32.h>

#include <SoftwareSerial.h>
#include <Sabertooth.h>

boolean arcRobot = true;
boolean useBumpers = true;
boolean useFallSensors = false;
boolean useIRSensors = true;
boolean useUSSensors = false;
boolean useBatterySensor = true;

const byte nLowerRelays = 4;
const byte nUpperRelays = 4;
const byte nLightsUpper = 3;
const byte nLightsLower = 3;
const byte nIRSensors = 7;
const byte nMotors = 2;
const byte nBumpers = 3;
const byte nFallSensors = 4;

// Define Left Encoder Pins
// Right Encoder A-2, Right Encoder B-3
const byte rightEncoderPins[] = {2, 3};

// Define Digital Pin for Ultrasonics
const byte usDigitalPin = 4;

// Define Motor Serial Tx Pin
const byte motorPin = 10;

// Define Max485 (Ultrasonic) Serial Pins
// Ultrasonic Tx-14, Ultrasonic Rx-15
const byte ultrasonicSerial[] = {14, 15};

// Define Right Encoder Pins
// Left Encoder A-18, Left Encoder B-19
const byte leftEncoderPins[] = {18, 19};

// Define Lower Relay Pins
// Motor Relay-22, Unused-23
// Infrared Relay-24, IMU/Fall Sensor Relay-25
const byte lowerRelayPins[] = {22, 23, 24, 25};


// Define Upper Relay Pins
// Slam Device Relay-26, Upper Lights Relay-27
// Lower Lights Relay-28, Unused-29
const byte upperRelayPins[] = {26, 27, 28, 29};

// Define Bumper Pins
// Center Bumper-32, Left Bumper-33, Right Bumper-34
const byte bumperPins[] = {32, 33, 34};

// Define Upper R, G, B Pins
// Upper Red-38, Upper Green-39, Upper Blue-40
const byte lightsUpperPins[] = {38, 39, 40};

// Define Lower R, G, B Pins
// Lower Red-41, Lower Green-42, Lower Blue-43
const byte lightsLowerPins[] = {41, 42, 43};

// Define Bluetooth Serial Pins
// Bluetooth Rx-48, Bluetooth Tx-49
const byte btSerialPins[] = {48, 49};

// Define Rear IR Sensor Pins
// Rear Left IR-A0, Front Right Outer IR-A1, Front Center IR-A2
// Front Left Outer IR-A3, Rear Right IR-A4,
// Front Right Inner IR-A5, Front Left Inner IR-A6
const byte irSensorPins[] = {A0, A1, A2, A3, A4, A5, A6};

// Define Voltage Sensor Pin
const byte voltageSensorPin = A8;

// Define FallSensorPins
// Front Right Fall-A12, Rear Right Fall-A13
// Front Left Fall-A14, Rear Left Fall-A15
const byte fallSensorPins[] = {A12, A13, A14, A15}; 


SoftwareSerial SWSerial(NOT_A_PIN, motorPin);
Sabertooth ST(128,SWSerial);


byte bumperValue = 0;
byte previousBumperValue = 0;
byte fallSensorValue = 0;
byte previousFallSensorValue = 0;
byte bumperVals[nBumpers] = {0, 0, 0};
int fallSensorVals[nFallSensors] = {0, 0, 0, 0};
int irSensorVals[nIRSensors] = {0, 0, 0, 0, 0, 0, 0};

long lOldEncoderPos  = -999;
long rOldEncoderPos  = -999;

int v1;
float v2;
float vout;
float r1 = 30000.0;
float r2 = 7500.0;
char battOutput[8];
long previousBatteryMillis = 0;
long batteryPubRate = 10000;
long previousEncoderMillis = 0;
long encoderPubRate = 50;
long previousIRMillis = 0;
long irPubRate = 250; 
long previousUSMillis = 0;
long usPubRate = 50; 

ros::NodeHandle  nh;

tango_msgs::bumpers bumpers_msg;
tango_msgs::lights lights_msg;
tango_msgs::relays relays_msg;
tango_msgs::fallsensors fallsensors_msg;
tango_msgs::motor_commands motor_commands_msg;
tango_msgs::wheel_encoders wheel_encoders_msg;
tango_msgs::infrareds infrareds_msg;
tango_msgs::ultrasonics ultrasonics_msg;
sensor_msgs::BatteryState batteryState_msg;

ros::Publisher rWheelEncoders("/wheel_encoders", &wheel_encoders_msg);
ros::Publisher rBumpers("/tango_msgs/bumpers", &bumpers_msg);
ros::Publisher rFallSensors("/tango_msgs/fallsensors", &fallsensors_msg);
ros::Publisher rIRSensors("/infrareds", &infrareds_msg);
//ros::Publisher rUSSensors("/tango_msgs/ultrasonics", &ultrasonics_msg);
ros::Publisher rBatteryState("/battery", &batteryState_msg);

void relayOnOff(byte relayNum, byte on=0){
  switch (relayNum){
    case 1:
      nh.logdebug("Changing state of motor controller power relay");
      if (on == 1){
        digitalWrite(lowerRelayPins[0], HIGH);
      } else if (on == 0){
        digitalWrite(lowerRelayPins[0], LOW);
      }
      break;
    case 2:
      nh.logdebug("Changing state of unused power relay");
      if (on == 1){
        digitalWrite(lowerRelayPins[1], HIGH);
      } else if (on == 0){
        digitalWrite(lowerRelayPins[1], LOW);
      }
      break;
    case 3:
      nh.logdebug("Changing state of infrared sensors power relay");
      if (on == 1){
        digitalWrite(lowerRelayPins[2], HIGH);
      } else if (on == 0){
        digitalWrite(lowerRelayPins[2], LOW);
      }
      break;
    case 4:
      nh.logdebug("Changing state of IMU and fall sensors power relay");
      if (on == 1){
        digitalWrite(lowerRelayPins[3], HIGH);
      } else if (on == 0){
        digitalWrite(lowerRelayPins[3], LOW);
      }
      break;
    case 5:
      nh.logdebug("Changing state of slam device power relay");
      if (on == 1){
        digitalWrite(upperRelayPins[0], HIGH);
      } else if (on == 0){
        digitalWrite(upperRelayPins[0], LOW);
      }
      break;
  }    
  if (arcRobot == false){
    switch (relayNum){
      case 6:
        nh.logdebug("Changing state of upper lights power relay");
        if (on == 1){
          digitalWrite(upperRelayPins[1], HIGH);
        } else if (on == 0){
          digitalWrite(upperRelayPins[1], LOW);
        }
        break;
      case 7:
        nh.logdebug("Changing state lower lights power relay");
        if (on == 1){
          digitalWrite(upperRelayPins[2], HIGH);
        } else if (on == 0){
          digitalWrite(upperRelayPins[2], LOW);
        }
        break;
      case 8:
        nh.logdebug("Changing state of unused power relay");
        if (on == 1){
          digitalWrite(upperRelayPins[3], HIGH);
        } else if (on == 0){
          digitalWrite(upperRelayPins[3], LOW);
        }
        break;
    }
  }
}

void rRelaysCb(const tango_msgs::relays& relays_msg){
  if (relays_msg.state == 1){
    if (arcRobot == false){
      relayOnOff(relays_msg.number, 1);
    } else if (arcRobot == true){
      relayOnOff(relays_msg.number, 0);
    }
  } else if (relays_msg.state == 0){
    if (arcRobot == false){
      relayOnOff(relays_msg.number, 0);
    } else if(arcRobot == true){
      relayOnOff(relays_msg.number, 1);
    }
  }  
}


void rMotorCmdsCb(const tango_msgs::motor_commands& motor_commands_msg){
  ST.motor(1, (int)motor_commands_msg.l_motor);
  ST.motor(2, (int)motor_commands_msg.r_motor);
}

ros::Subscriber<tango_msgs::relays> rRelays("/tango_msgs/relays", &rRelaysCb);
ros::Subscriber<tango_msgs::lights> rLights("/tango_msgs/lights", &rLightsCb);
ros::Subscriber<tango_msgs::motor_commands> rMotorCommands("/motor_commands", &rMotorCmdsCb);

void readBatteryVoltage(){
  if((millis()-previousBatteryMillis) > batteryPubRate){
    v1 = analogRead(voltageSensorPin);
    v2 = ((v1*5.0)/1024.0);
    vout = v2/(r2/(r1+r2));
    dtostrf(vout, 6, 2, battOutput);
    nh.logdebug("Battery Voltage = ");
    nh.logdebug(battOutput);
    if (vout < 11.2){
      nh.logwarn("Battery voltage is below 11.2V. Please charge battery soon");
      nh.logwarn("Consider shutting down tango to charge the main battery");
    } else if (vout < 10.8){
      nh.logerror("Battery voltage critial. Stopping all operations!");
      nh.logerror("Please shutdown tango and charge the main battery");
    }
    batteryState_msg.voltage = vout;
    batteryState_msg.header.stamp = nh.now();
    rBatteryState.publish(&batteryState_msg);
    previousBatteryMillis = millis();
  }
}




void haltRobot(){
  ST.stop();
}



void setupBatterySensorRos(){
  nh.loginfo("Populating battery message parameters");
  batteryState_msg.capacity = 6.9;
  batteryState_msg.design_capacity = 7;
  batteryState_msg.power_supply_technology = 0;
  batteryState_msg.present = "true";
}

void setupMotorController(){
  nh.loginfo("Initialising motor controller");
  if (arcRobot == false){
   relayOnOff(1,1);
  } else if (arcRobot == true){
   relayOnOff(1,0);
  }
  delay(2000);
  SWSerial.begin(38400);
  haltRobot();
  nh.loginfo("Motor controller initialisation complete");
}

void setupIRSensorsRelay(){
  if (useIRSensors == true){
    nh.loginfo("Setting up infrared sensors");
    delay(1000);
    if (arcRobot == false){
      relayOnOff(2, 1);
    } else if(arcRobot == true){
      relayOnOff(3, 0);
    }
  }
}

void setupFallSensorsRelay(){
  if (useFallSensors == true){
    nh.loginfo("Setting up fall sensors");
    delay(1000);
    if (arcRobot == false){
      relayOnOff(3, 1);
    } else if(arcRobot == true){
      relayOnOff(2, 0);
    }
  }  
}

void setup(){
  nh.getHardware()->setBaud(115200);
  nh.initNode();
  nh.subscribe(rRelays);
  nh.subscribe(rLights);
  nh.subscribe(rMotorCommands);
  nh.advertise(rBumpers);
  nh.advertise(rFallSensors);
  nh.advertise(rIRSensors);
  //nh.advertise(rUSSensors);
  nh.advertise(rWheelEncoders);
  nh.advertise(rBatteryState);
  while(!nh.connected()) {
    nh.spinOnce();
  }
  delay(1000);
  for (byte i=0; i<nBumpers; i++){
    pinMode(bumperPins[i], INPUT_PULLUP);
  }
  for (byte i=0; i<nFallSensors; i++){
    pinMode(fallSensorPins[i], INPUT);
  }
  for (byte i=0; i<nIRSensors; i++){
    pinMode(irSensorPins[i], INPUT);
  }
  for (byte i=0; i<nLowerRelays; i++){
    pinMode(lowerRelayPins[i], OUTPUT);
    if (arcRobot == false){
      relayOnOff(i, 0);
    } else if (arcRobot == true){
      relayOnOff(i, 1);
    }
  }
  for (byte i=0; i<nUpperRelays; i++){
    pinMode(upperRelayPins[i], OUTPUT);
    if (arcRobot == false){
      relayOnOff(i+nUpperRelays, 0);
    } else if (arcRobot == true){
      relayOnOff(i+nUpperRelays, 1);
    }
  }
  if (arcRobot == false){
    for (byte i=0; i<nLightsUpper; i ++){
      pinMode(lightsUpperPins[i], OUTPUT);
      pinMode(lightsLowerPins[i], OUTPUT);
      digitalWrite(lightsUpperPins[i], LOW);
      digitalWrite(lightsLowerPins[i], LOW);
    }
  }
  pinMode(voltageSensorPin, INPUT);
  nh.loginfo("Initialising tango robot");
  setupIRSensorsRelay();
  setupInfraredSensorsRos();
  setupFallSensorsRelay();
  //setupUltrasonicSensorsRos();
  setupBatterySensorRos();
  setupMotorController();
  nh.loginfo("Tango robot initialisation complete");
}

void loop()
{ 
  if (useBumpers == true){
    checkBumpers();
  }
  if (useFallSensors == true){
    readFallSensors();
  }
  if (useIRSensors == true){
    readIRSensors();
  }
  if (useUSSensors == true){
    readUSSensors();
  }
  if (useBatterySensor == true){
    readBatteryVoltage();
  }
  readEncoders();
  nh.spinOnce();
}
