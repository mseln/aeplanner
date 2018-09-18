#include <ros/ros.h>
#include <aeplanner/aeplanner.h>

int main(int argc, char** argv)
{
  ros::init(argc, argv, "aeplanner");
  ros::NodeHandle nh;
  
  aeplanner::AEPlanner aeplanner(nh);

  ros::spin();
  return 0;
}
