#ifndef _AEPVIZ_H_
#define _AEPVIZ_H_

#include <string>

#include <visualization_msgs/Marker.h>
#include <visualization_msgs/MarkerArray.h>
#include <eigen3/Eigen/Dense>
#include <tf/transform_datatypes.h>

#include <aeplanner/data_structures.h>

namespace aeplanner {
  visualization_msgs::MarkerArray createRRTMarkerArray(std::shared_ptr<RRTNode> root, double lambda);
  void recurse(std::shared_ptr<RRTNode> node, visualization_msgs::MarkerArray * marker_array, int * id, double lambda);

  visualization_msgs::Marker createNodeMarker(std::shared_ptr<RRTNode> node, int id, std::string frame_id);
  visualization_msgs::Marker createEdgeMarker(std::shared_ptr<RRTNode> node, int id, std::string frame_id, double lambda);
}
#endif
