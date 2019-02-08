
#ifndef _RRT_H_
#define _RRT_H_

#include <ros/ros.h>
#include <visualization_msgs/Marker.h>

#include <octomap/octomap.h>
#include <octomap_msgs/conversions.h>

#include <eigen3/Eigen/Dense>

#include <rrtplanner/rrtAction.h>
#include <actionlib/server/simple_action_server.h>

#include <kdtree/kdtree.h>
#include <nav_msgs/Path.h>

namespace aeplanner_ns
{
struct RrtNode
{
  Eigen::Vector3d pos;
  RrtNode *parent;
  std::vector<RrtNode *> children;
  ~RrtNode()
  {
    for (typename std::vector<RrtNode *>::iterator node_it = children.begin();
         node_it != children.end(); ++node_it)
    {
      delete (*node_it);
      (*node_it) = NULL;
    }
  }

  double cost()
  {
    if (parent)
      return (pos - parent->pos).norm() + parent->cost();
    return 0;
  }
};

class Rrt
{
public:
  Rrt(const ros::NodeHandle &nh);
  void octomapCallback(const octomap_msgs::Octomap &msg);

  void execute(const rrtplanner::rrtGoalConstPtr &goal);
  void visualizeGoals(std::vector<geometry_msgs::Pose> goals);
  void visualizeNode(geometry_msgs::Point pos, int id = 0);
  void visualizePose(geometry_msgs::Pose pose, int id = 0);
  void visualizeEdge(RrtNode *node, int id = 0);
  void visualizePath(RrtNode *node);

  Eigen::Vector3d sample();
  RrtNode *chooseParent(kdtree *kd_tree, Eigen::Vector3d, double l);
  void rewire(kdtree *kd_tree, RrtNode *new_node, double l, double r, double r_os);
  Eigen::Vector3d getNewPos(Eigen::Vector3d sampled, Eigen::Vector3d parent, double l);
  bool collisionLine(Eigen::Vector3d p1, Eigen::Vector3d p2, double r);
  RrtNode *addNodeToTree(kdtree *kd_tree, RrtNode *parent, Eigen::Vector3d new_pos);
  RrtNode *getGoal(kdtree *goal_tree, RrtNode *new_node, double l, double r, double r_os);
  nav_msgs::Path getBestPath(std::vector<RrtNode *> goals);
  std::vector<geometry_msgs::Pose> checkIfGoalReached(kdtree *goal_tree, RrtNode *new_node, double l, double r, double r_os);

private:
  ros::NodeHandle nh_;
  std::shared_ptr<octomap::OcTree> ot_;

  ros::Subscriber octomap_sub_;
  actionlib::SimpleActionServer<rrtplanner::rrtAction> as_;

  std::string frame_id_;

  ros::Publisher path_pub_;
  double min_nodes_;
  double bounding_radius_;
  double bounding_overshoot_;
  double extension_range_;
  std::vector<double> boundary_min_;
  std::vector<double> boundary_max_;
};

float CylTest_CapsFirst(const octomap::point3d &pt1,
                        const octomap::point3d &pt2,
                        float lsq, float rsq, const octomap::point3d &pt);
} // namespace aeplanner_ns

#endif
