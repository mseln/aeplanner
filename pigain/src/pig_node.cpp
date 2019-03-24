#include <ros/ros.h>
#include <pigain/pig.h>

int main(int argc, char** argv)
{
  ros::init(argc, argv, "pig");
  ros::NodeHandle nh;

  pig::PIG pig(nh);

  ros::spin();
  return 0;
}