#include "MyRobot.hpp"
#include "controller_manager/controller_manager.h"
#include "hardware_interface/actuator_state_interface.h"
#include "wheelchair_msgs/wheelVels.h"

#include <ros/callback_queue.h>
int main(int argc, char** argv)
{
  ros::init(argc, argv, "test_iface_node");
  ros::NodeHandle nh;
  ros::CallbackQueue queue;
  nh.setCallbackQueue(&queue);

  double leftWheelCmd; //variable for left wheel diff drive
  double rightWheelCmd; //variable for right wheel diff drive


  MyRobot robot;
  controller_manager::ControllerManager cm(&robot,nh);
  ros::Publisher motors_cmds_pub = nh.advertise<wheelchair_msgs::wheelVels>("/wheelchair_robot/motor_commands", 1000); //publish topic to arduino rosserial


  ros::AsyncSpinner spinner(4, &queue);
  spinner.start();

  ros::Time ts = ros::Time::now();

  ros::Rate rate(50);
  while (ros::ok())
  {
     ros::Duration d = ros::Time::now() - ts;
     ts = ros::Time::now();
     robot.read();
     cm.update(ts, d);
     robot.write();
     leftWheelCmd = robot.getLeftWheelCmd(); //get left wheel cmd
     rightWheelCmd = robot.getRightWheelCmd(); //get right wheel cmd
     wheelchair_msgs::wheelVels msgs; //initialise msgs for motor commands
     msgs.leftVel = leftWheelCmd; //append left motor val
     msgs.rightVel = rightWheelCmd; //append right motor val
     motors_cmds_pub.publish(msgs); //publish motor commands

     rate.sleep();
  }

  spinner.stop();

  return 0;
}
