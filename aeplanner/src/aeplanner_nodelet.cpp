#include <aeplanner/aeplanner_nodelet.h>

#include <pluginlib/class_list_macros.h>
PLUGINLIB_EXPORT_CLASS(aeplanner::AEPlannerNodelet, nodelet::Nodelet)

namespace aeplanner
{
void AEPlannerNodelet::onInit()
{
  NODELET_DEBUG("Initializing nodelet AEPlanner...");

  ros::NodeHandle& nh = getMTNodeHandle();  // getNodeHandle();
  aeplanner_ = new AEPlanner(nh);
}
AEPlannerNodelet::~AEPlannerNodelet()
{
  delete aeplanner_;
}
}  // namespace aeplanner
