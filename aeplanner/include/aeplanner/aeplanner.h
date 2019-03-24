#ifndef AEPLANNER_H
#define AEPLANNER_H

#include <ros/ros.h>

#include <geometry_msgs/PoseStamped.h>
#include <geometry_msgs/PoseArray.h>

#include <tf/transform_listener.h>

#include <octomap/octomap.h>
#include <octomap_msgs/conversions.h>

#include <eigen3/Eigen/Dense>

#include <aeplanner/data_structures.h>
#include <aeplanner/param.h>
#include <aeplanner/Reevaluate.h>

#include <aeplanner/aeplanner_viz.h>
#include <visualization_msgs/MarkerArray.h>

#include <aeplanner/aeplannerAction.h>
#include <actionlib/server/simple_action_server.h>

#include <aeplanner_msgs/Node.h>
#include <aeplanner_msgs/Query.h>
#include <aeplanner_msgs/BestNode.h>

#include <dynamic_reconfigure/server.h>
#include <nav_msgs/Path.h>

namespace aeplanner
{
class AEPlanner
{
typedef std::pair<point, std::shared_ptr<RRTNode>> value;
typedef boost::geometry::index::rtree<value, boost::geometry::index::rstar<16>> value_rtree;

private:
  ros::NodeHandle nh_;
  actionlib::SimpleActionServer<aeplanner::aeplannerAction> as_;

  Params params_;

  // Current state of agent (x, y, z, yaw)
  Eigen::Vector4d current_state_;
  bool current_state_initialized_;

  // Keep track of the best node and its score
  std::shared_ptr<RRTNode> best_node_;
  std::shared_ptr<RRTNode> best_branch_root_;

  std::shared_ptr<octomap::OcTree> ot_;

  // kd tree for finding nearest neighbours
  value_rtree rtree_;

  // Subscribers
  ros::Subscriber octomap_sub_;
  ros::Subscriber agent_pose_sub_;

  // Publishers
  ros::Publisher rrt_marker_pub_;
  ros::Publisher gain_pub_;

  // Services
  ros::ServiceClient best_node_client_;
  ros::ServiceClient gp_query_client_;
  ros::ServiceServer reevaluate_server_;

  // Service server callback
  bool reevaluate(aeplanner::Reevaluate::Request& req,
                  aeplanner::Reevaluate::Response& res);

  // ---------------- Initialization ----------------
  std::shared_ptr<RRTNode> initialize();
  void initializeKDTreeWithPreviousBestBranch(std::shared_ptr<RRTNode> root);
  void reevaluatePotentialInformationGainRecursive(std::shared_ptr<RRTNode> node);

  // ---------------- Expand RRT Tree ----------------
  void expandRRT();

  Eigen::Vector4d sampleNewPoint();
  bool isInsideBoundaries(Eigen::Vector4d point);
  bool collisionLine(Eigen::Vector4d p1, Eigen::Vector4d p2, double r);
  std::shared_ptr<RRTNode> chooseParent(std::shared_ptr<RRTNode> node, double l);
  void rewire(std::shared_ptr<RRTNode> new_node, double l, double r, double r_os);
  Eigen::Vector4d restrictDistance(Eigen::Vector4d nearest, Eigen::Vector4d new_pos);

  std::pair<double, double> getGain(std::shared_ptr<RRTNode> node);
  std::pair<double, double> gainCubature(Eigen::Vector4d state);

  // ---------------- Helpers ----------------
  //
  void publishEvaluatedNodesRecursive(std::shared_ptr<RRTNode> node);

  geometry_msgs::Pose vecToPose(Eigen::Vector4d state);

  float CylTest_CapsFirst(const octomap::point3d& pt1, const octomap::point3d& pt2,
                          float lsq, float rsq, const octomap::point3d& pt);

  // ---------------- Frontier ----------------
  geometry_msgs::PoseArray getFrontiers();

public:
  AEPlanner(const ros::NodeHandle& nh);

  void execute(const aeplanner::aeplannerGoalConstPtr& goal);

  void octomapCallback(const octomap_msgs::Octomap& msg);
  void agentPoseCallback(const geometry_msgs::PoseStamped& msg);
};

}  // namespace aeplanner

#endif
