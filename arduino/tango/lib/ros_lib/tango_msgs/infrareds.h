#ifndef _ROS_tango_msgs_infrareds_h
#define _ROS_tango_msgs_infrareds_h

#include <stdint.h>
#include <string.h>
#include <stdlib.h>
#include "ros/msg.h"
#include "sensor_msgs/Range.h"

namespace tango_msgs
{

  class infrareds : public ros::Msg
  {
    public:
      typedef sensor_msgs::Range _front_c_ir_type;
      _front_c_ir_type front_c_ir;
      typedef sensor_msgs::Range _front_r_inner_ir_type;
      _front_r_inner_ir_type front_r_inner_ir;
      typedef sensor_msgs::Range _front_r_outer_ir_type;
      _front_r_outer_ir_type front_r_outer_ir;
      typedef sensor_msgs::Range _rear_r_ir_type;
      _rear_r_ir_type rear_r_ir;
      typedef sensor_msgs::Range _rear_l_ir_type;
      _rear_l_ir_type rear_l_ir;
      typedef sensor_msgs::Range _front_l_outer_ir_type;
      _front_l_outer_ir_type front_l_outer_ir;
      typedef sensor_msgs::Range _front_l_inner_ir_type;
      _front_l_inner_ir_type front_l_inner_ir;

    infrareds():
      front_c_ir(),
      front_r_inner_ir(),
      front_r_outer_ir(),
      rear_r_ir(),
      rear_l_ir(),
      front_l_outer_ir(),
      front_l_inner_ir()
    {
    }

    virtual int serialize(unsigned char *outbuffer) const
    {
      int offset = 0;
      offset += this->front_c_ir.serialize(outbuffer + offset);
      offset += this->front_r_inner_ir.serialize(outbuffer + offset);
      offset += this->front_r_outer_ir.serialize(outbuffer + offset);
      offset += this->rear_r_ir.serialize(outbuffer + offset);
      offset += this->rear_l_ir.serialize(outbuffer + offset);
      offset += this->front_l_outer_ir.serialize(outbuffer + offset);
      offset += this->front_l_inner_ir.serialize(outbuffer + offset);
      return offset;
    }

    virtual int deserialize(unsigned char *inbuffer)
    {
      int offset = 0;
      offset += this->front_c_ir.deserialize(inbuffer + offset);
      offset += this->front_r_inner_ir.deserialize(inbuffer + offset);
      offset += this->front_r_outer_ir.deserialize(inbuffer + offset);
      offset += this->rear_r_ir.deserialize(inbuffer + offset);
      offset += this->rear_l_ir.deserialize(inbuffer + offset);
      offset += this->front_l_outer_ir.deserialize(inbuffer + offset);
      offset += this->front_l_inner_ir.deserialize(inbuffer + offset);
     return offset;
    }

    const char * getType(){ return "tango_msgs/infrareds"; };
    const char * getMD5(){ return "0b88a4ce49ea32cafa291df9c12dea24"; };

  };

}
#endif
