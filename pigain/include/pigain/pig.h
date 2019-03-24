#ifndef PIGAIN_PIG_H
#define PIGAIN_PIG_H

#include <ros/ros.h>

#include <geometry_msgs/PoseStamped.h>
#include <visualization_msgs/Marker.h>
#include <visualization_msgs/MarkerArray.h>

#include <aeplanner_msgs/BestNode.h>
#include <aeplanner_msgs/Node.h>
#include <aeplanner_msgs/Query.h>

#include <boost/geometry.hpp>
#include <boost/geometry/index/rtree.hpp>

#include <tf2/utils.h>
#include <eigen3/Eigen/Dense>

namespace pig
{
struct Node
{
  geometry_msgs::Point position;
  double yaw;
  double gain;

  Node()
  {
  }

  Node(const geometry_msgs::Point& position, double yaw, double gain) : position(position), yaw(yaw), gain(gain)
  {
  }

  Node(const aeplanner_msgs::Node& node)
    : position(node.pose.pose.position), yaw(tf2::getYaw(node.pose.pose.orientation)), gain(node.gain)
  {
  }

  bool operator==(const Node& rhs) const
  {
    return yaw == rhs.yaw && gain == rhs.gain && position.x == rhs.position.x && position.y == rhs.position.y &&
           position.z == rhs.position.z;
  }
};

class PIG
{
private:
  // Node handle
  ros::NodeHandle& nh_;

  // Service servers
  ros::ServiceServer gp_srv_;
  ros::ServiceServer best_node_srv_;

  // Service clients
  ros::ServiceClient reevaluate_client_;

  // Subscribers
  ros::Subscriber gain_sub_;
  ros::Subscriber pose_sub_;

  // Publishers
  ros::Publisher marker_pub_;
  ros::Publisher mean_pub_;
  ros::Publisher sigma_pub_;

  // Timers
  ros::Timer marker_pub_timer_;
  ros::Timer mean_and_sigma_pub_timer_;
  ros::Timer reevaluate_timer_;

  // Pose
  bool pose_recieved_;
  geometry_msgs::PoseStamped pose_;

  // Rtree
  typedef boost::geometry::model::point<double, 3, boost::geometry::cs::cartesian> point;
  typedef boost::geometry::model::box<point> box;
  typedef std::tuple<point, Node, unsigned int> value;

  boost::geometry::index::rtree<value, boost::geometry::index::rstar<16>> rtree_;
  unsigned int id_;

  // Bounding box
  point bbx_min_;
  point bbx_max_;

  // GP
  double hyper_l_;
  double hyper_sigma_f_;
  double hyper_sigma_n_;

  double resolution_;
  double range_;

public:
  PIG(ros::NodeHandle& nh);

private:
  bool gpQueryCallback(aeplanner_msgs::Query::Request& req, aeplanner_msgs::Query::Response& res);

  bool bestNodeCallback(aeplanner_msgs::BestNode::Request& req, aeplanner_msgs::BestNode::Response& res);

  void gainCallback(const aeplanner_msgs::Node::ConstPtr& msg);

  void poseCallback(const geometry_msgs::PoseStamped::ConstPtr& msg);

  void evaluateCallback(const ros::TimerEvent& event);

  void reevaluateCallback(const ros::TimerEvent& event);

  void rvizCallback(const ros::TimerEvent& event);

  visualization_msgs::Marker pointToMarker(unsigned int id, Eigen::Vector3d point, double v = 0, double a = 0);

  visualization_msgs::Marker nodeToMarker(unsigned int id, const Node node);
};
}  // namespace pig

#endif  // PIGAIN_GP_H