#include <EEPROM.h>
#include <Servo.h>
#include "TeensyServo.h"
#include "configuration.h"
#include "protocol.h"
#include <ros.h>
#include <inmoov_msgs/MotorCommand.h>
#include <inmoov_msgs/MotorParameter.h>
#include <inmoov_msgs/MotorStatus.h>

#define LED 13

void commandCb(const inmoov_msgs::MotorCommand&);
void getParameter(const inmoov_msgs::MotorParameter::Request&, inmoov_msgs::MotorParameter::Response&);

int i, j;
int updateMillis;
int commands;
int servoCount = 0;

TeensyServo *tServo[20];

char joint[2] = " ";
byte bus = 0;

int startMillis;

ros::NodeHandle  nh;

inmoov_msgs::MotorCommand command_msg;
inmoov_msgs::MotorParameter parameter_msg;
inmoov_msgs::MotorStatus status_msg;

// Publishers and subscribers for ROS
ros::Publisher motorstatus("motorstatus", &status_msg);
ros::Subscriber<inmoov_msgs::MotorCommand> motorcommand("motorcommand", &commandCb);
ros::ServiceServer<inmoov_msgs::MotorParameter::Request, inmoov_msgs::MotorParameter::Response> server("motorparameter", &getParameter);


const bool heartbeats[] = {1, 0, 1, 0, 0, 0, 0, 0};

TeensyServo* getServo(int pin) {
  for(int i = 0; i < servoCount; i++) {
    if(tServo[i]->getServoPin() == pin) {
      return tServo[i];
    }
  }
  return 0;
}

void getParameter(const inmoov_msgs::MotorParameter::Request & req, inmoov_msgs::MotorParameter::Response & res) {
  byte id = req.id;
  byte parameter = req.parameter;
  float value = 0.0;
  TeensyServo* servo = getServo(id);

  if(servo != 0) {
    switch (parameter) {
      case P_GOALPOSITION:
        value = servo->getGoal();
        break;

      case P_MINANGLE:
        value = servo->getMinAngle();
        break;

      case P_MAXANGLE:
        value = servo->getMaxAngle();
        break;

      case P_MINPULSE:
        value = (float)servo->getMinPulse();
        break;

      case P_MAXPULSE:
        value = (float)servo->getMaxPulse();
        break;

      case P_MINSENSOR:
        value = (float)servo->getMinSensor();
        break;

      case P_MAXSENSOR:
        value = (float)servo->getMaxSensor();
        break;

      case P_CALIBRATED:
        value = servo->getCalibrated();
        break;

      case P_PRESENTPOSITION:
        value = servo->readPositionAngle();
        break;

      case P_SENSORRAW:
        value = (float)servo->readPositionRaw();
        break;

      case P_MOVING:
        value = (float)servo->getMoving();
        break;

      case P_PRESENTSPEED:
        value = servo->readPresentSpeed();
        break;

      case P_SMOOTH:
        value = (float)servo->getSmooth();
        break;

      case P_GOALSPEED:
        value = servo->getMaxSpeed();
        break;

      case P_ENABLE:
        value = (float)servo->getEnabled();
        break;

      case P_POWER:
        value = (float)servo->getPower();
        break;
    }
  }

  res.data = value;
}

void commandCb( const inmoov_msgs::MotorCommand& command_msg) {
  byte id = command_msg.id;
  byte parameter = command_msg.parameter;
  float value = command_msg.value;
  TeensyServo* servo = getServo(id);

  if(parameter == P_SERVO_INIT) {
   // nh.loginfo("new Servo: " + id);
    // int v = (int)value;
    // int servoPin = v & 0x3FF;
    // int sensorPin = (v & 0xFFC00) >> 10;
    // sensorPin = (sensorPin == 0x3FF)?-1:sensorPin;
    if(servoCount < 12) {
      tServo[servoCount] = new TeensyServo(id, int(value));
      servoCount++;
    }
  }


  if(servo != 0) {
    switch (parameter) {
      case P_GOALPOSITION:
        servo->setGoal(value);
        //String string = "Goal Position = " + String(tServo[id]->commandPulse);
        //nh.loginfo(String(tServo[id]->commandPulse));
        break;

      case P_MINANGLE:
        servo->setMinAngle(value);
        break;

      case P_MAXANGLE:
        servo->setMaxAngle(value);
        break;

      case P_MINPULSE:
        servo->setMinPulse(value);
        break;

      case P_MAXPULSE:
        servo->setMaxPulse(value);
        break;

      case P_ENABLE:
        servo->setEnabled(value);
        break;

      case P_CALIBRATED:
        servo->setCalibrated(value);
        break;

      case P_MINSENSOR:
        servo->setMinSensor(value);
        break;

      case P_MAXSENSOR:
        servo->setMaxSensor(value);
        break;

      case P_SMOOTH:
        servo->setSmooth(value);
        break;

      case P_GOALSPEED:
        servo->setMaxSpeed(value);
        break;
    }
  }

}

void setupADC() {
  // if it's not the teensy don't set ADC resolution
  #if !defined(ARDUINO_AVR_MEGA2560)
    analogReadResolution(12);
  #endif
  analogReference(EXTERNAL);
}

void updateServos() {
  for (i = 0; i < servoCount; i++) {
    tServo[i]->update();
  }
}

void setupServos() {
//   tServo[servoCount++] = new TeensyServo(0, A8);
//   tServo[1] = new TeensyServo(1, A9);
//   tServo[2] = new TeensyServo(2, A10);
//   tServo[3] = new TeensyServo(3, A11);
//   tServo[4] = new TeensyServo(4, A7);
//   tServo[5] = new TeensyServo(5, A6);
//   tServo[6] = new TeensyServo(6, A5);
//   tServo[7] = new TeensyServo(7, A4);
//   tServo[8] = new TeensyServo(8, A3);
//   tServo[9] = new TeensyServo(9, A2);
//   tServo[10] = new TeensyServo(10, A1);
//   tServo[11] = new TeensyServo(11, A0);
}

byte generateChecksum() {
  return 0;
}

void setup() {
  pinMode(LED, OUTPUT);
  digitalWrite(LED, 1);

  nh.initNode();
  nh.advertise(motorstatus);
  nh.subscribe(motorcommand);
  nh.advertiseService(server);

  while (!nh.connected() ) {
    nh.spinOnce();
  }

  setupADC();
  delay(1);

  setupServos();

  Serial.begin(115200);

  startMillis = millis();
  updateMillis = millis();
  commands = 0;

  nh.loginfo("Setup Complete!!!");
}

void loop() {
  updateServos();
  digitalWrite(LED, heartbeats[((millis() >> 7) & 7)]);

  if ((millis() - updateMillis) >= UPDATEPERIOD) {
    for (int servo = 0; servo < servoCount; servo++) {

      status_msg.joint        = joint;
      status_msg.bus          = bus;
      status_msg.id           = tServo[servo]->getServoPin();
      status_msg.goal         = tServo[servo]->getGoal();
      status_msg.position     = tServo[servo]->readPositionAngle();
      status_msg.presentspeed = tServo[servo]->readPresentSpeed();
      status_msg.moving       = tServo[servo]->getMoving();
      //status_msg.posraw       = tServo[servo]->sampleDuration;
      status_msg.posraw       = tServo[servo]->readPositionRaw();
      status_msg.enabled      = tServo[servo]->getEnabled();
      status_msg.power        = tServo[servo]->getPower();

      nh.spinOnce();

      motorstatus.publish(&status_msg);

      nh.spinOnce();
    }

    updateMillis = millis();
    nh.spinOnce();

    commands = 0;
  }
  nh.spinOnce();
}
