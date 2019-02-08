#include <ros/ros.h>

#include <aeplanner/param.h>

namespace aeplanner
{
  Params readParams()
  {
    // FIXME namespaces
    Params params;
    std::string ns = ros::this_node::getNamespace();
    params.hfov = 60;
    if (!ros::param::get(ns + "/camera/horizontal_fov", params.hfov)) {
      ROS_WARN_STREAM("No horizontal fov specified. Default: " << params.hfov);
    }
    params.vfov = 45;
    if (!ros::param::get(ns + "/camera/vertical_fov", params.vfov)) {
      ROS_WARN_STREAM("No vertical fov specified. Default: " << params.vfov);
    }
    params.dr = 0.1;
    if (!ros::param::get("/octomap_server/resolution", params.dr)) {
      ROS_WARN_STREAM("Could not read octomap resolution. Looking for /octomap_server/resolution.");
      ROS_WARN_STREAM("Using resolution specified by param file instead");
    }
    else if (!ros::param::get(ns + "/raycast/dr", params.dr)) {
      ROS_WARN_STREAM("No dr specified. Default: " << params.dr);
    }
    params.dphi = 10;
    if (!ros::param::get(ns + "/raycast/dphi", params.dphi)) {
      ROS_WARN_STREAM("No dphi specified. Default: " << params.dphi);
    }
    params.dtheta = 10;
    if (!ros::param::get(ns + "/raycast/dtheta", params.dtheta)) {
      ROS_WARN_STREAM("No dtheta specified. Default: " << params.dtheta);
    }
    params.lambda = 0.5;
    if (!ros::param::get(ns + "/aep/gain/lambda", params.lambda)) {
      ROS_WARN_STREAM("No lambda specified. Default: " << params.lambda);
    }
    params.extension_range = 1.0;
    if (!ros::param::get(ns + "/aep/tree/extension_range", params.extension_range)) {
      ROS_WARN_STREAM("No extension_range specified. Default: " << params.extension_range);
    }
    params.max_sampling_radius = 10.0;
    if (!ros::param::get(ns + "/aep/tree/max_sampling_radius", params.max_sampling_radius)) {
      ROS_WARN_STREAM("No max_sampling_radius specified. Default: " << params.max_sampling_radius);
    }
    params.sigma_thresh = 0.2;
    if (!ros::param::get(ns + "/aep/gain/sigma_thresh", params.sigma_thresh)) {
      ROS_WARN_STREAM("No sigma_thresh specified. Default: " << params.sigma_thresh);
    }
    params.init_iterations = 15;
    if (!ros::param::get(ns + "/aep/tree/initial_iterations", params.init_iterations)) {
      ROS_WARN_STREAM("No init_iterations specified. Default: " << params.init_iterations);
    }
    params.r_max = 4.0;
    if (!ros::param::get(ns + "/aep/gain/r_max", params.r_max)) {
      ROS_WARN_STREAM("No r_max specified. Default: " << params.r_max);
    }
    params.r_min = 0.5;
    if (!ros::param::get(ns + "/aep/gain/r_min", params.r_min)) {
      ROS_WARN_STREAM("No r_min specified. Default: " << params.r_min);
    }
    if (!ros::param::get(ns + "/boundary/min", params.boundary_min)) {
      ROS_WARN_STREAM("No boundary/min specified.");
    }
    if (!ros::param::get(ns + "/boundary/max", params.boundary_max)) {
      ROS_WARN_STREAM("No boundary/max specified.");
    }
    params.bounding_radius;
    if (!ros::param::get(ns + "/system/bbx/r", params.bounding_radius)) {
      ROS_WARN_STREAM("No /system/bbx/r specified. Default: " << params.bounding_radius);
    }
    params.cutoff_iterations = 200;
    if (!ros::param::get(ns + "/aep/tree/cutoff_iterations", params.cutoff_iterations)) {
      ROS_WARN_STREAM("No /aep/tree/cutoff_iterations specified. Default: " << params.cutoff_iterations);
    }
    params.zero_gain = 0.0;
    if (!ros::param::get(ns + "/aep/gain/zero", params.zero_gain)) {
      ROS_WARN_STREAM("No /aep/gain/zero specified. Default: " << params.zero_gain);
    }
    params.d_overshoot_ = 0.5;
    if (!ros::param::get(ns + "/system/bbx/overshoot", params.d_overshoot_)) {
      ROS_WARN_STREAM("No /system/bbx/overshoot specified. Default: " << params.d_overshoot_);
    }
    params.world_frame = "world";
    if (!ros::param::get(ns + "/world_frame", params.world_frame)) {
      ROS_WARN_STREAM("No /world_frame specified. Default: " << params.world_frame);
    }
    params.robot_frame = "base_link";
    if (!ros::param::get(ns + "/robot_frame", params.robot_frame)) {
      ROS_WARN_STREAM("No /robot_frame specified. Default: " << params.robot_frame);
    }
    params.visualize_tree = false;
    if (!ros::param::get(ns + "/visualize_tree", params.visualize_tree)) {
      ROS_WARN_STREAM("No /visualize_tree specified. Default: " << params.visualize_tree);
    }
    params.visualize_rays = false;
    if (!ros::param::get(ns + "/visualize_rays", params.visualize_rays)) {
      ROS_WARN_STREAM("No /visualize_rays specified. Default: " << params.visualize_rays);
    }
    params.visualize_exploration_area = false;
    if (!ros::param::get(ns + "/visualize_exploration_area", params.visualize_exploration_area)) {
      ROS_WARN_STREAM("No /visualize_exploration_area specified. Default: " << params.visualize_exploration_area);
    }

    return params;
  }
}
