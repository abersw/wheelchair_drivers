#ifndef _ROS_tango_msgs_ultrasonics_h
#define _ROS_tango_msgs_ultrasonics_h

#include <stdint.h>
#include <string.h>
#include <stdlib.h>
#include "ros/msg.h"
#include "sensor_msgs/Range.h"

namespace tango_msgs
{

  class ultrasonics : public ros::Msg
  {
    public:
      typedef sensor_msgs::Range _front_c_us_type;
      _front_c_us_type front_c_us;
      typedef sensor_msgs::Range _front_r_us_type;
      _front_r_us_type front_r_us;
      typedef sensor_msgs::Range _rear_r_us_type;
      _rear_r_us_type rear_r_us;
      typedef sensor_msgs::Range _rear_c_us_type;
      _rear_c_us_type rear_c_us;
      typedef sensor_msgs::Range _rear_l_us_type;
      _rear_l_us_type rear_l_us;
      typedef sensor_msgs::Range _front_l_us_type;
      _front_l_us_type front_l_us;

    ultrasonics():
      front_c_us(),
      front_r_us(),
      rear_r_us(),
      rear_c_us(),
      rear_l_us(),
      front_l_us()
    {
    }

    virtual int serialize(unsigned char *outbuffer) const
    {
      int offset = 0;
      offset += this->front_c_us.serialize(outbuffer + offset);
      offset += this->front_r_us.serialize(outbuffer + offset);
      offset += this->rear_r_us.serialize(outbuffer + offset);
      offset += this->rear_c_us.serialize(outbuffer + offset);
      offset += this->rear_l_us.serialize(outbuffer + offset);
      offset += this->front_l_us.serialize(outbuffer + offset);
      return offset;
    }

    virtual int deserialize(unsigned char *inbuffer)
    {
      int offset = 0;
      offset += this->front_c_us.deserialize(inbuffer + offset);
      offset += this->front_r_us.deserialize(inbuffer + offset);
      offset += this->rear_r_us.deserialize(inbuffer + offset);
      offset += this->rear_c_us.deserialize(inbuffer + offset);
      offset += this->rear_l_us.deserialize(inbuffer + offset);
      offset += this->front_l_us.deserialize(inbuffer + offset);
     return offset;
    }

    const char * getType(){ return "tango_msgs/ultrasonics"; };
    const char * getMD5(){ return "f1ea0c59e4063cb70034eba24ff497d5"; };

  };

}
#endif
