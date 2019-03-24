
#ifndef _RRT_H_
#define _RRT_H_

#include <ros/ros.h>
#include <visualization_msgs/Marker.h>

#include <octomap/octomap.h>
#include <octomap_msgs/conversions.h>

#include <eigen3/Eigen/Dense>

#include <aeplanner_msgs/rrtAction.h>
#include <actionlib/server/simple_action_server.h>

#include <nav_msgs/Path.h>

#include <boost/geometry.hpp>
#include <boost/geometry/index/rtree.hpp>

namespace aeplanner_ns
{
// Rtree
typedef boost::geometry::model::point<double, 3, boost::geometry::cs::cartesian> point;
typedef boost::geometry::model::box<point> box;

typedef boost::geometry::index::rtree<point, boost::geometry::index::rstar<16>>
    point_rtree;

struct RrtNode
{
  Eigen::Vector3d pos;
  std::shared_ptr<RrtNode> parent;
  std::vector<std::shared_ptr<RrtNode>> children;

  double cost()
  {
    if (parent)
      return (pos - parent->pos).norm() + parent->cost();
    return 0;
  }
};

class Rrt
{
  typedef std::pair<point, std::shared_ptr<RrtNode>> value;
  typedef boost::geometry::index::rtree<value, boost::geometry::index::rstar<16>>
      value_rtree;

public:
  Rrt(const ros::NodeHandle& nh);
  void octomapCallback(const octomap_msgs::Octomap& msg);

  void execute(const aeplanner_msgs::rrtGoalConstPtr& goal);
  void visualizeGoals(std::vector<geometry_msgs::Pose> goals);
  void visualizeNode(geometry_msgs::Point pos, int id = 0);
  void visualizePose(geometry_msgs::Pose pose, int id = 0);
  void visualizeEdge(std::shared_ptr<RrtNode> node, int id = 0);
  void visualizePath(std::shared_ptr<RrtNode> node);

  Eigen::Vector3d sample();
  std::shared_ptr<RrtNode> chooseParent(const value_rtree& rtree, Eigen::Vector3d,
                                        double l);
  void rewire(const value_rtree& rtree, std::shared_ptr<RrtNode> new_node, double l,
              double r, double r_os);
  Eigen::Vector3d getNewPos(Eigen::Vector3d sampled, Eigen::Vector3d parent, double l);
  bool collisionLine(Eigen::Vector3d p1, Eigen::Vector3d p2, double r);
  std::shared_ptr<RrtNode> addNodeToTree(value_rtree* rtree,
                                         std::shared_ptr<RrtNode> parent,
                                         Eigen::Vector3d new_pos);
  std::shared_ptr<RrtNode> getGoal(const point_rtree& goal_tree,
                                   std::shared_ptr<RrtNode> new_node, double l, double r,
                                   double r_os);
  nav_msgs::Path getBestPath(std::vector<std::shared_ptr<RrtNode>> goals);
  std::vector<geometry_msgs::Pose> checkIfGoalReached(const point_rtree& goal_tree,
                                                      std::shared_ptr<RrtNode> new_node,
                                                      double l, double r, double r_os);

private:
  ros::NodeHandle nh_;
  std::shared_ptr<octomap::OcTree> ot_;

  ros::Subscriber octomap_sub_;
  actionlib::SimpleActionServer<aeplanner_msgs::rrtAction> as_;

  std::string frame_id_;

  ros::Publisher path_pub_;
  double min_nodes_;
  double bounding_radius_;
  double bounding_overshoot_;
  double extension_range_;
  std::vector<double> boundary_min_;
  std::vector<double> boundary_max_;
};

float CylTest_CapsFirst(const octomap::point3d& pt1, const octomap::point3d& pt2,
                        float lsq, float rsq, const octomap::point3d& pt);
}  // namespace aeplanner_ns

#endif
