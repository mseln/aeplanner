#include <aeplanner/aeplanner_viz.h>

namespace aeplanner {
  visualization_msgs::MarkerArray createRRTMarkerArray(RRTNode * root, double lambda){
    int id = 0;
    visualization_msgs::MarkerArray marker_array;
    recurse(root, &marker_array, &id, lambda);

    return marker_array;
  }
  void recurse(RRTNode * node, visualization_msgs::MarkerArray * marker_array, int * id, double lambda){
    for(std::vector<RRTNode*>::iterator child_it  = node->children_.begin();
                                        child_it != node->children_.end(); ++child_it){
      RRTNode * child = (*child_it);
      if(child) recurse(child, marker_array, id, lambda);
      marker_array->markers.push_back(createEdgeMarker(child, (*id), "map", lambda)); // FIXME read frame id from config
      marker_array->markers.push_back(createNodeMarker(child, (*id)++, "map"));
    }
  }

  visualization_msgs::Marker createNodeMarker(RRTNode * node, int id, std::string frame_id){
    visualization_msgs::Marker marker;
    marker.header.stamp = ros::Time::now();
    marker.header.seq = id;
    marker.header.frame_id = frame_id;
    marker.id = id;
    marker.ns = "nodes";
    marker.type = visualization_msgs::Marker::ARROW;
    marker.action = visualization_msgs::Marker::ADD;
    marker.pose.position.x = node->state_[0];
    marker.pose.position.y = node->state_[1];
    marker.pose.position.z = node->state_[2];
    tf::Quaternion quat;
    quat.setEuler(0.0, 0.0, node->state_[3]);
    marker.pose.orientation.x = quat.x();
    marker.pose.orientation.y = quat.y();
    marker.pose.orientation.z = quat.z();
    marker.pose.orientation.w = quat.w();
    marker.scale.x = std::max(node->gain_ / 72.0, 0.05);
    marker.scale.y = 0.1;
    marker.scale.z = 0.1;
    marker.color.r = 167.0 / 255.0;
    marker.color.g = 167.0 / 255.0;
    marker.color.b = 0.0;
    marker.color.a = 1.0;
    marker.lifetime = ros::Duration(10.0);
    marker.frame_locked = false;

    return marker;
  }

  visualization_msgs::Marker createEdgeMarker(RRTNode * node, int id, std::string frame_id, double lambda){
    visualization_msgs::Marker marker;
    marker.header.stamp = ros::Time::now();
    marker.header.seq = id;
    marker.header.frame_id = frame_id;
    marker.id = id;
    marker.ns = "edges";
    marker.type = visualization_msgs::Marker::ARROW;
    marker.action = visualization_msgs::Marker::ADD;
    marker.pose.position.x = node->parent_->state_[0];
    marker.pose.position.y = node->parent_->state_[1];
    marker.pose.position.z = node->parent_->state_[2];
    Eigen::Quaternion<double> q;
    Eigen::Vector3d init(1.0, 0.0, 0.0);
    Eigen::Vector3d dir(node->state_[0] - node->parent_->state_[0],
                        node->state_[1] - node->parent_->state_[1],
                        node->state_[2] - node->parent_->state_[2]);
    q.setFromTwoVectors(init, dir);
    q.normalize();
    marker.pose.orientation.x = q.x();
    marker.pose.orientation.y = q.y();
    marker.pose.orientation.z = q.z();
    marker.pose.orientation.w = q.w();
    marker.scale.x = dir.norm();
    marker.scale.y = 0.03;
    marker.scale.z = 0.03;
    marker.color.r = node->score(lambda) / 60.0;
    marker.color.g = 0.0;
    marker.color.b = 1.0;
    marker.color.a = 1.0;
    marker.lifetime = ros::Duration(10.0);
    marker.frame_locked = false;

    return marker;
  }

}
