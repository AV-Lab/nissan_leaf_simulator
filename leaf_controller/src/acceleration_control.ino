#include <ros.h>
#include <std_msgs/Float32.h>


ros::NodeHandle nh;

void messageCb(const std_msgs::Float32& msg)
{
  String velocity_value= msg.data;
  const char *logging_char = velocity_value.c_str();
  nh.loginfo(logging_char);
}

ros::Subscriber<std_msgs::Float64> sub("velocity", &messageCb);


void setup()
{
    nh.initNode();
    nh.subscribe(sub);
}

void loop()
{
    nh.spinOnce();
    delay(20);
}

