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

  double leftWheelCmd;
  double rightWheelCmd;


  MyRobot robot;
  controller_manager::ControllerManager cm(&robot,nh);
  ros::Publisher motors_cmds_pub = nh.advertise<wheelchair_msgs::wheelVels>("motor_cmds", 1000);


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
     leftWheelCmd = robot.getLeftWheelCmd();
     rightWheelCmd = robot.getRightWheelCmd();
     wheelchair_msgs::wheelVels msgs;
     msgs.leftVel = leftWheelCmd;
     msgs.rightVel = rightWheelCmd;
     motors_cmds_pub.publish(msgs);

     rate.sleep();
  }

  spinner.stop();

  return 0;
}
