#ifndef _ROS_tango_msgs_lights_h
#define _ROS_tango_msgs_lights_h

#include <stdint.h>
#include <string.h>
#include <stdlib.h>
#include "ros/msg.h"

namespace tango_msgs
{

  class lights : public ros::Msg
  {
    public:
      typedef int8_t _tiers_type;
      _tiers_type tiers;
      typedef int8_t _colour_type;
      _colour_type colour;

    lights():
      tiers(0),
      colour(0)
    {
    }

    virtual int serialize(unsigned char *outbuffer) const
    {
      int offset = 0;
      union {
        int8_t real;
        uint8_t base;
      } u_tiers;
      u_tiers.real = this->tiers;
      *(outbuffer + offset + 0) = (u_tiers.base >> (8 * 0)) & 0xFF;
      offset += sizeof(this->tiers);
      union {
        int8_t real;
        uint8_t base;
      } u_colour;
      u_colour.real = this->colour;
      *(outbuffer + offset + 0) = (u_colour.base >> (8 * 0)) & 0xFF;
      offset += sizeof(this->colour);
      return offset;
    }

    virtual int deserialize(unsigned char *inbuffer)
    {
      int offset = 0;
      union {
        int8_t real;
        uint8_t base;
      } u_tiers;
      u_tiers.base = 0;
      u_tiers.base |= ((uint8_t) (*(inbuffer + offset + 0))) << (8 * 0);
      this->tiers = u_tiers.real;
      offset += sizeof(this->tiers);
      union {
        int8_t real;
        uint8_t base;
      } u_colour;
      u_colour.base = 0;
      u_colour.base |= ((uint8_t) (*(inbuffer + offset + 0))) << (8 * 0);
      this->colour = u_colour.real;
      offset += sizeof(this->colour);
     return offset;
    }

    const char * getType(){ return "tango_msgs/lights"; };
    const char * getMD5(){ return "500b425a9f72b157796dedb4d149b278"; };

  };

}
#endif
