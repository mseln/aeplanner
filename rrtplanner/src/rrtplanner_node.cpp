#include <rrtplanner/rrt.h>

int main(int argc, char** argv)
{
  ros::init(argc, argv, "rrt");
  ros::NodeHandle nh;

  aeplanner_ns::Rrt rrt(nh);

  ros::spin();
  return 0;
}
