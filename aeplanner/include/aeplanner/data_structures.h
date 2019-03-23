#ifndef DATA_STRUCTURES_H
#define DATA_STRUCTURES_H

#include <ros/ros.h>
#include <octomap/OcTree.h>

#include <boost/geometry.hpp>
#include <boost/geometry/index/rtree.hpp>

namespace aeplanner
{
typedef boost::geometry::model::point<double, 3, boost::geometry::cs::cartesian> point;
typedef boost::geometry::model::box<point> box;
typedef boost::geometry::index::rtree<point, boost::geometry::index::rstar<16>> point_rtree;

class RRTNode
{
public:
  Eigen::Vector4d state_;
  std::shared_ptr<RRTNode> parent_;
  std::vector<std::shared_ptr<RRTNode>> children_;
  double gain_;
  bool gain_explicitly_calculated_;

  RRTNode() : parent_(NULL), gain_(0.0), gain_explicitly_calculated_(false)
  {
  }

  std::shared_ptr<RRTNode> getCopyOfParentBranch()
  {
    std::shared_ptr<RRTNode> current_node = std::make_shared<RRTNode>(*this);
    std::shared_ptr<RRTNode> new_node;
    std::shared_ptr<RRTNode> new_child_node = NULL;

    while (current_node)
    {
      new_node = std::make_shared<RRTNode>();
      new_node->state_ = current_node->state_;
      new_node->gain_ = current_node->gain_;
      new_node->gain_explicitly_calculated_ = current_node->gain_explicitly_calculated_;
      new_node->parent_ = NULL;

      if (new_child_node)
      {
        new_node->children_.push_back(new_child_node);
        new_child_node->parent_ = new_node;
      }

      current_node = current_node->parent_;
      new_child_node = new_node;
    }

    return new_node;
  }

  double score(double lambda)
  {
    if (this->parent_)
      return this->parent_->score(lambda) +
             this->gain_ * exp(-lambda * this->distance(this->parent_));
    else
      return this->gain_;
  }

  double cost()
  {
    if (this->parent_)
      return this->distance(this->parent_) + this->parent_->cost();
    else
      return 0;
  }

  double distance(std::shared_ptr<RRTNode> other)
  {
    Eigen::Vector3d p3(this->state_[0], this->state_[1], this->state_[2]);
    Eigen::Vector3d q3(other->state_[0], other->state_[1], other->state_[2]);
    return (p3 - q3).norm();
  }
};
}  // namespace aeplanner

#endif
