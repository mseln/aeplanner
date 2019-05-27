#include <pigain/gp.h>
#include <pigain/pig.h>
#include <ros/ros.h>
#include <fstream>

#include <aeplanner_msgs/Reevaluate.h>

namespace pig
{
PIG::PIG(ros::NodeHandle& nh)
  : nh_(nh)
  , gp_srv_(nh_.advertiseService("gp_query_server", &PIG::gpQueryCallback, this))
  , best_node_srv_(nh_.advertiseService("best_node_server", &PIG::bestNodeCallback, this))
  , reevaluate_client_(nh_.serviceClient<aeplanner_msgs::Reevaluate>("reevaluate"))
  , gain_sub_(nh_.subscribe("gain_node", 100, &PIG::gainCallback, this))
  , pose_sub_(nh_.subscribe("pose", 10, &PIG::poseCallback, this))
  , marker_pub_(nh_.advertise<visualization_msgs::MarkerArray>("pig_markers", 10, true))
  , mean_pub_(nh_.advertise<visualization_msgs::MarkerArray>("mean_markers", 10, true))
  , sigma_pub_(nh_.advertise<visualization_msgs::MarkerArray>("sigma_markers", 10, true))
  , marker_pub_timer_(nh_.createTimer(ros::Duration(1), &PIG::rvizCallback, this))
  , mean_and_sigma_pub_timer_(
        nh.createTimer(ros::Duration(5), &PIG::evaluateCallback, this))
  , reevaluate_timer_(nh_.createTimer(ros::Duration(10), &PIG::reevaluateCallback, this))
  , pose_recieved_(false)
  , hyper_l_(1.0)
  , hyper_sigma_f_(1.0)
  , hyper_sigma_n_(0.1)
{
  // ---------- Get environment boundaries ----------
  resolution_ = nh_.param("visualize/resolution", 1.0);
  if (!nh_.getParam("aep/gain/r_max", range_))
  {
    ROS_WARN("Range max parameter not specified");
    range_ = 8;
    ROS_WARN("Defaulting to %f m...", range_);
  }
  range_ *= 2;  // Because of reasons TODO: Explain

  std::vector<double> min;
  std::vector<double> max;
  if (!nh_.getParam("boundary/min", min) || min.size() != 3)
  {
    ROS_WARN("Min boundary parameter not specified or incorrect");
    min.push_back(-100);
    min.push_back(-100);
    min.push_back(0);
    ROS_WARN("Defaulting to (%f, %f, %f)", min[0], min[1], min[2]);
  }
  if (!nh_.getParam("boundary/max", max) || max.size() != 3)
  {
    ROS_WARN("Max boundary parameter not specified or incorrect");
    max.push_back(100);
    max.push_back(100);
    max.push_back(3);
    ROS_WARN("Defaulting to (%f, %f, %f)", max[0], max[1], max[2]);
  }

  bbx_min_ = point(min[0], min[1], min[2]);
  bbx_max_ = point(max[0], max[1], max[2]);
}

bool PIG::gpQueryCallback(aeplanner_msgs::Query::Request& req,
                          aeplanner_msgs::Query::Response& res)
{

  box query_box(point(req.point.x - 2, req.point.y - 2, req.point.z - 2),
                point(req.point.x + 2, req.point.y + 2, req.point.z + 2));
  std::vector<value> hits;
  rtree_.query(boost::geometry::index::intersects(query_box), std::back_inserter(hits));
  std::vector<value> nearest;
  rtree_.query(boost::geometry::index::nearest(query_box, 1),
               std::back_inserter(nearest));

  Eigen::VectorXd y(hits.size());
  Eigen::Matrix<double, Eigen::Dynamic, 3> x(hits.size(), 3);

  for (size_t i = 0; i < hits.size(); ++i)
  {
    y(i) = std::get<1>(hits[i]).gain;
    geometry_msgs::Point position = std::get<1>(hits[i]).position;
    x(i, 0) = position.x;
    x(i, 1) = position.y;
    x(i, 2) = position.z;
  }

  double yaw = 0;
  for (value item : nearest)
  {
    yaw = std::get<1>(item).yaw;
  }

  if (y.size() == 0)
  {
    res.mu = 0;
    res.sigma = 1;
    return true;
  }

  Eigen::Matrix<double, Eigen::Dynamic, 3> x_star(1, 3);
  x_star(0, 0) = req.point.x;
  x_star(0, 1) = req.point.y;
  x_star(0, 2) = req.point.z;

  std::pair<double, double> gp_response =
      gp(y, x, x_star, hyper_l_, hyper_sigma_f_, hyper_sigma_n_);

  res.mu = gp_response.first;
  res.sigma = gp_response.second;
  res.yaw = yaw;

  return true;
}

bool PIG::bestNodeCallback(aeplanner_msgs::BestNode::Request& req,
                           aeplanner_msgs::BestNode::Response& res)
{
  box query_box(bbx_min_, bbx_max_);
  std::vector<value> hits;
  rtree_.query(boost::geometry::index::intersects(query_box), std::back_inserter(hits));

  double highest_gain = -1;
  for (value item : hits)
  {
    geometry_msgs::Point position = std::get<1>(item).position;
    double gain = std::get<1>(item).gain;
    if (gain > req.threshold)
    {
      res.best_node.push_back(position);
    }
    highest_gain = std::max(gain, highest_gain);
  }

  res.gain = highest_gain;
  return true;
}

void PIG::gainCallback(const aeplanner_msgs::Node::ConstPtr& msg)
{
  point p(msg->pose.pose.position.x, msg->pose.pose.position.y,
          msg->pose.pose.position.z);

  rtree_.insert(std::make_tuple(p, Node(*msg), id_++));
}

void PIG::poseCallback(const geometry_msgs::PoseStamped::ConstPtr& msg)
{
  pose_ = *msg;
  pose_recieved_ = true;
}

void PIG::evaluateCallback(const ros::TimerEvent& event)
{
  if (true || (mean_pub_.getNumSubscribers() == 0 && sigma_pub_.getNumSubscribers() == 0))
  {
    return;
  }

  box query_box(bbx_min_, bbx_max_);
  std::vector<value> hits;
  rtree_.query(boost::geometry::index::intersects(query_box), std::back_inserter(hits));

  Eigen::VectorXd y(hits.size());
  Eigen::Matrix<double, Eigen::Dynamic, 3> x(hits.size(), 3);

  for (size_t i = 0; i < hits.size(); ++i)
  {
    y(i) = std::get<1>(hits[i]).gain;
    geometry_msgs::Point position = std::get<1>(hits[i]).position;
    x(i, 0) = position.x;
    x(i, 1) = position.y;
    x(i, 2) = position.z;
  }

  Eigen::Matrix<double, Eigen::Dynamic, 3> x_star(
      int((bbx_max_.get<0>() - bbx_min_.get<0>()) / resolution_) +
          int((bbx_max_.get<1>() - bbx_min_.get<1>()) / resolution_) +
          int((bbx_max_.get<2>() - bbx_min_.get<2>()) / resolution_),
      3);
  size_t index = 0;
  for (double x = bbx_min_.get<0>(); x < bbx_max_.get<0>(); x += resolution_)
  {
    for (double y = bbx_min_.get<1>(); y < bbx_max_.get<1>(); y += resolution_)
    {
      for (double z = bbx_min_.get<2>(); z < bbx_max_.get<2>(); z += resolution_)
      {
        x_star(index, 0) = x;
        x_star(index, 1) = y;
        x_star(index, 2) = z;
        ++index;
      }
    }
  }

  std::pair<double, double> gp_response =
      gp(y, x, x_star, hyper_l_, hyper_sigma_f_, hyper_sigma_n_);

  visualization_msgs::MarkerArray mean_markers;
  visualization_msgs::MarkerArray sigma_markers;
  for (size_t i = 0; i < x_star.rows(); ++i)
  {
    mean_markers.markers.push_back(pointToMarker(
        i, x_star.row(i), gp_response.first, std::max(1.0 - gp_response.second, 0.0)));
    sigma_markers.markers.push_back(
        pointToMarker(i, x_star.row(i), gp_response.second * 2));
  }

  mean_pub_.publish(mean_markers);
  sigma_pub_.publish(sigma_markers);
}

void PIG::reevaluateCallback(const ros::TimerEvent& event)
{
  if (!pose_recieved_)
  {
    ROS_WARN("No position received yet...");
    ROS_WARN("Make sure that 'pose' has been correctly mapped and that it is being "
             "published");
    return;
  }

  geometry_msgs::Point position = pose_.pose.position;

  box query_box(point(position.x - range_, position.y - range_, position.z - range_),
                point(position.x + range_, position.y + range_, position.z + range_));
  std::vector<value> hits;
  rtree_.query(boost::geometry::index::intersects(query_box), std::back_inserter(hits));

  std::vector<value> reevaluate_list;
  std::vector<geometry_msgs::Point> reevaluate_position_list;
  for (value item : hits)
  {
    if (std::get<1>(item).gain > 0.1)
    {
      reevaluate_list.push_back(item);
      reevaluate_position_list.push_back(std::get<1>(item).position);
    }
  }

  aeplanner_msgs::Reevaluate srv;
  srv.request.point = reevaluate_position_list;
  if (!reevaluate_client_.call(srv))
  {
    ROS_ERROR("Calling reevaluate service failed");
    return;
  }

  for (size_t i = 0; i < reevaluate_list.size(); ++i)
  {
    value item = reevaluate_list[i];
    Node node(std::get<1>(item));
    node.gain = srv.response.gain[i];
    node.yaw = srv.response.yaw[i];

    rtree_.remove(item);
    rtree_.insert(std::make_tuple(std::get<0>(item), node, std::get<2>(item)));
  }
}

void PIG::rvizCallback(const ros::TimerEvent& event)
{
  if (marker_pub_.getNumSubscribers() == 0)
  {
    return;
  }

  visualization_msgs::MarkerArray markers;

  box query_box(bbx_min_, bbx_max_);
  std::vector<value> hits;
  boost::geometry::index::rtree<value, boost::geometry::index::rstar<16>> rtree = rtree_;
  rtree.query(boost::geometry::index::intersects(query_box), std::back_inserter(hits));

  for (value item : hits)
  {
    markers.markers.push_back(nodeToMarker(std::get<2>(item), std::get<1>(item)));
  }

  marker_pub_.publish(markers);
}

visualization_msgs::Marker PIG::pointToMarker(unsigned int id, Eigen::Vector3d point,
                                              double v, double a)
{
  visualization_msgs::Marker marker;
  marker.header.frame_id = "map";
  marker.type = marker.CUBE;
  marker.action = marker.ADD;
  marker.id = id;
  marker.scale.x = resolution_;
  marker.scale.y = resolution_;
  marker.scale.z = 0.1;
  marker.color.r = v;
  marker.color.g = 0;
  marker.color.b = 0.5;
  marker.color.a = a;
  marker.pose.orientation.w = 1.0;
  marker.pose.position.x = point[0];
  marker.pose.position.y = point[1];
  marker.pose.position.z = point[2];
  marker.lifetime = ros::Duration(10);

  return marker;
}

visualization_msgs::Marker PIG::nodeToMarker(unsigned int id, const Node node)
{
  visualization_msgs::Marker marker;
  marker.header.frame_id = "map";
  marker.type = marker.SPHERE;
  marker.action = marker.ADD;
  marker.id = id;
  marker.scale.x = 0.4;
  marker.scale.y = 0.4;
  marker.scale.z = 0.4;
  marker.color.r = node.gain / 32;
  marker.color.g = 0.0;
  marker.color.b = 0.5;
  marker.color.a = 1.0;
  marker.pose.orientation.w = 1.0;
  marker.pose.position.x = node.position.x;
  marker.pose.position.y = node.position.y;
  marker.pose.position.z = node.position.z;
  marker.lifetime = ros::Duration(3);

  return marker;
}
}  // namespace pig
