#ifndef _ROS_tango_msgs_wheel_encoders_h
#define _ROS_tango_msgs_wheel_encoders_h

#include <stdint.h>
#include <string.h>
#include <stdlib.h>
#include "ros/msg.h"

namespace tango_msgs
{

  class wheel_encoders : public ros::Msg
  {
    public:
      typedef int16_t _l_count_type;
      _l_count_type l_count;
      typedef int16_t _r_count_type;
      _r_count_type r_count;

    wheel_encoders():
      l_count(0),
      r_count(0)
    {
    }

    virtual int serialize(unsigned char *outbuffer) const
    {
      int offset = 0;
      union {
        int16_t real;
        uint16_t base;
      } u_l_count;
      u_l_count.real = this->l_count;
      *(outbuffer + offset + 0) = (u_l_count.base >> (8 * 0)) & 0xFF;
      *(outbuffer + offset + 1) = (u_l_count.base >> (8 * 1)) & 0xFF;
      offset += sizeof(this->l_count);
      union {
        int16_t real;
        uint16_t base;
      } u_r_count;
      u_r_count.real = this->r_count;
      *(outbuffer + offset + 0) = (u_r_count.base >> (8 * 0)) & 0xFF;
      *(outbuffer + offset + 1) = (u_r_count.base >> (8 * 1)) & 0xFF;
      offset += sizeof(this->r_count);
      return offset;
    }

    virtual int deserialize(unsigned char *inbuffer)
    {
      int offset = 0;
      union {
        int16_t real;
        uint16_t base;
      } u_l_count;
      u_l_count.base = 0;
      u_l_count.base |= ((uint16_t) (*(inbuffer + offset + 0))) << (8 * 0);
      u_l_count.base |= ((uint16_t) (*(inbuffer + offset + 1))) << (8 * 1);
      this->l_count = u_l_count.real;
      offset += sizeof(this->l_count);
      union {
        int16_t real;
        uint16_t base;
      } u_r_count;
      u_r_count.base = 0;
      u_r_count.base |= ((uint16_t) (*(inbuffer + offset + 0))) << (8 * 0);
      u_r_count.base |= ((uint16_t) (*(inbuffer + offset + 1))) << (8 * 1);
      this->r_count = u_r_count.real;
      offset += sizeof(this->r_count);
     return offset;
    }

    const char * getType(){ return "tango_msgs/wheel_encoders"; };
    const char * getMD5(){ return "fb7f414fcc222ece9a601cf18b112bc5"; };

  };

}
#endif
