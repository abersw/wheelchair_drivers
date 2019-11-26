#ifndef _ROS_tango_msgs_motor_commands_h
#define _ROS_tango_msgs_motor_commands_h

#include <stdint.h>
#include <string.h>
#include <stdlib.h>
#include "ros/msg.h"

namespace tango_msgs
{

  class motor_commands : public ros::Msg
  {
    public:
      typedef float _l_motor_type;
      _l_motor_type l_motor;
      typedef float _r_motor_type;
      _r_motor_type r_motor;

    motor_commands():
      l_motor(0),
      r_motor(0)
    {
    }

    virtual int serialize(unsigned char *outbuffer) const
    {
      int offset = 0;
      union {
        float real;
        uint32_t base;
      } u_l_motor;
      u_l_motor.real = this->l_motor;
      *(outbuffer + offset + 0) = (u_l_motor.base >> (8 * 0)) & 0xFF;
      *(outbuffer + offset + 1) = (u_l_motor.base >> (8 * 1)) & 0xFF;
      *(outbuffer + offset + 2) = (u_l_motor.base >> (8 * 2)) & 0xFF;
      *(outbuffer + offset + 3) = (u_l_motor.base >> (8 * 3)) & 0xFF;
      offset += sizeof(this->l_motor);
      union {
        float real;
        uint32_t base;
      } u_r_motor;
      u_r_motor.real = this->r_motor;
      *(outbuffer + offset + 0) = (u_r_motor.base >> (8 * 0)) & 0xFF;
      *(outbuffer + offset + 1) = (u_r_motor.base >> (8 * 1)) & 0xFF;
      *(outbuffer + offset + 2) = (u_r_motor.base >> (8 * 2)) & 0xFF;
      *(outbuffer + offset + 3) = (u_r_motor.base >> (8 * 3)) & 0xFF;
      offset += sizeof(this->r_motor);
      return offset;
    }

    virtual int deserialize(unsigned char *inbuffer)
    {
      int offset = 0;
      union {
        float real;
        uint32_t base;
      } u_l_motor;
      u_l_motor.base = 0;
      u_l_motor.base |= ((uint32_t) (*(inbuffer + offset + 0))) << (8 * 0);
      u_l_motor.base |= ((uint32_t) (*(inbuffer + offset + 1))) << (8 * 1);
      u_l_motor.base |= ((uint32_t) (*(inbuffer + offset + 2))) << (8 * 2);
      u_l_motor.base |= ((uint32_t) (*(inbuffer + offset + 3))) << (8 * 3);
      this->l_motor = u_l_motor.real;
      offset += sizeof(this->l_motor);
      union {
        float real;
        uint32_t base;
      } u_r_motor;
      u_r_motor.base = 0;
      u_r_motor.base |= ((uint32_t) (*(inbuffer + offset + 0))) << (8 * 0);
      u_r_motor.base |= ((uint32_t) (*(inbuffer + offset + 1))) << (8 * 1);
      u_r_motor.base |= ((uint32_t) (*(inbuffer + offset + 2))) << (8 * 2);
      u_r_motor.base |= ((uint32_t) (*(inbuffer + offset + 3))) << (8 * 3);
      this->r_motor = u_r_motor.real;
      offset += sizeof(this->r_motor);
     return offset;
    }

    const char * getType(){ return "tango_msgs/motor_commands"; };
    const char * getMD5(){ return "3d152722070634c3e085a8ead9a10227"; };

  };

}
#endif
