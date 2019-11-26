#ifndef _ROS_tango_msgs_bumpers_h
#define _ROS_tango_msgs_bumpers_h

#include <stdint.h>
#include <string.h>
#include <stdlib.h>
#include "ros/msg.h"

namespace tango_msgs
{

  class bumpers : public ros::Msg
  {
    public:
      typedef int8_t _state_type;
      _state_type state;

    bumpers():
      state(0)
    {
    }

    virtual int serialize(unsigned char *outbuffer) const
    {
      int offset = 0;
      union {
        int8_t real;
        uint8_t base;
      } u_state;
      u_state.real = this->state;
      *(outbuffer + offset + 0) = (u_state.base >> (8 * 0)) & 0xFF;
      offset += sizeof(this->state);
      return offset;
    }

    virtual int deserialize(unsigned char *inbuffer)
    {
      int offset = 0;
      union {
        int8_t real;
        uint8_t base;
      } u_state;
      u_state.base = 0;
      u_state.base |= ((uint8_t) (*(inbuffer + offset + 0))) << (8 * 0);
      this->state = u_state.real;
      offset += sizeof(this->state);
     return offset;
    }

    const char * getType(){ return "tango_msgs/bumpers"; };
    const char * getMD5(){ return "a33bed68685ae53bd39b0a9242210752"; };

  };

}
#endif
