#ifndef DATA_STRUCTURES_H
#define DATA_STRUCTURES_H

#include <ros/ros.h>
#include <octomap/OcTree.h>

namespace aeplanner
{
class RRTNode
{
public:
  Eigen::Vector4d state_;
  RRTNode* parent_;
  std::vector<RRTNode*> children_;
  double gain_;
  bool gain_explicitly_calculated_;

  RRTNode() : parent_(NULL), gain_(0.0), gain_explicitly_calculated_(false)
  {
  }
  ~RRTNode()
  {
    for (typename std::vector<RRTNode*>::iterator node_it = children_.begin();
         node_it != children_.end(); ++node_it)
    {
      delete (*node_it);
      (*node_it) = NULL;
    }
  }

  RRTNode* getCopyOfParentBranch()
  {
    RRTNode* current_node = this;
    RRTNode* current_child_node = NULL;
    RRTNode* new_node;
    RRTNode* new_child_node = NULL;

    while (current_node)
    {
      new_node = new RRTNode();
      new_node->state_ = current_node->state_;
      new_node->gain_ = current_node->gain_;
      new_node->gain_explicitly_calculated_ = current_node->gain_explicitly_calculated_;
      new_node->parent_ = NULL;

      if (new_child_node)
      {
        new_node->children_.push_back(new_child_node);
        new_child_node->parent_ = new_node;
      }

      current_child_node = current_node;
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

  double cost(std::shared_ptr<octomap::OcTree> ot, Eigen::Vector4d current_state,
              double lambda, double max_distance, bool safety_first)
  {
    if (this->parent_)
    {
      double closest_distance =
          getDistanceToClosestOccupiedBounded(ot, current_state, 20);
      double distance_gain = 1;
      if (safety_first)
      {
        distance_gain = std::exp(-lambda * (closest_distance - max_distance));
      }
      else
      {
        distance_gain = std::exp(-lambda * (max_distance - closest_distance));
      }
      return (this->distance(this->parent_) * std::fmax(distance_gain, 1)) +
             this->parent_->cost(ot, current_state, lambda, max_distance, safety_first);
    }

    return 0;
  }

  double getDistanceToClosestOccupiedBounded(std::shared_ptr<octomap::OcTree> ot,
                                             Eigen::Vector4d current_state,
                                             double max_distance)
  {
    // TODO: OPTIMIZE. Start from current_state and move outwards

    octomap::point3d distance_point(max_distance, max_distance, 0);
    octomap::point3d oc_point(current_state[0], current_state[1], current_state[2]);

    double min_distance = max_distance;

    for (octomap::OcTree::leaf_bbx_iterator
             it = ot->begin_leafs_bbx(oc_point - distance_point,
                                      oc_point + distance_point),
             it_end = ot->end_leafs_bbx();
         it != it_end; ++it)
    {
      if (ot->isNodeOccupied(*it))
      {
        min_distance = std::min(min_distance, (oc_point - it.getCoordinate()).norm());
      }
    }

    return min_distance;
  }

  double distance(RRTNode* other)
  {
    Eigen::Vector3d p3(this->state_[0], this->state_[1], this->state_[2]);
    Eigen::Vector3d q3(other->state_[0], other->state_[1], other->state_[2]);
    return (p3 - q3).norm();
  }
};
}  // namespace aeplanner

#endif
