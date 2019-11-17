#ifndef _ROS_tango_msgs_relays_h
#define _ROS_tango_msgs_relays_h

#include <stdint.h>
#include <string.h>
#include <stdlib.h>
#include "ros/msg.h"

namespace tango_msgs
{

  class relays : public ros::Msg
  {
    public:
      typedef int8_t _number_type;
      _number_type number;
      typedef int8_t _state_type;
      _state_type state;

    relays():
      number(0),
      state(0)
    {
    }

    virtual int serialize(unsigned char *outbuffer) const
    {
      int offset = 0;
      union {
        int8_t real;
        uint8_t base;
      } u_number;
      u_number.real = this->number;
      *(outbuffer + offset + 0) = (u_number.base >> (8 * 0)) & 0xFF;
      offset += sizeof(this->number);
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
      } u_number;
      u_number.base = 0;
      u_number.base |= ((uint8_t) (*(inbuffer + offset + 0))) << (8 * 0);
      this->number = u_number.real;
      offset += sizeof(this->number);
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

    const char * getType(){ return "tango_msgs/relays"; };
    const char * getMD5(){ return "49090d479dd01f9ca70dee7a4573cca6"; };

  };

}
#endif
