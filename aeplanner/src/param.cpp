#include <ros/ros.h>

#include <aeplanner/param.h>

namespace aeplanner
{
  Params readParams()
  {
    Params params;
    std::string ns = ros::this_node::getNamespace();
    bool ret = true;
    params.hfov = 60;
    if (!ros::param::get(ns + "/camera/horizontal_fov", params.hfov)) {
      ROS_WARN("No horizontal fov specified. Looking for %s. Default is 90 deg.",
          (ns + "/camera/horizontal_fov").c_str());
    }
    params.vfov = 45;
    if (!ros::param::get(ns + "/camera/vertical_fov", params.vfov)) {
      ROS_WARN("No vertical fov specified. Looking for %s. Default is 45 deg.",
          (ns + "/camera/vertical_fov").c_str());
    }
    params.dr = 0.1;
    if (!ros::param::get("/octomap_server/resolution", params.dr)) {
      ROS_WARN("Could not read octomap resolution. Looking for /octomap_server/resolution.");
      ROS_WARN("Using resolution specified by param file instead");
    }
    else if (!ros::param::get(ns + "/raycast/dr", params.dr)) {
      ROS_WARN("No dr specified. Looking for %s. Default is 0.1 m.",
          (ns + "/raycast/dr").c_str());
    }
    params.dphi = 10;
    if (!ros::param::get(ns + "/raycast/dphi", params.dphi)) {
      ROS_WARN("No dphi specified. Looking for %s. Default is 10 deg.",
          (ns + "/raycast/dphi").c_str());
    }
    params.dtheta = 10;
    if (!ros::param::get(ns + "/raycast/dtheta", params.dtheta)) {
      ROS_WARN("No dtheta specified. Looking for %s. Default is 10 deg.",
          (ns + "/raycast/dtheta").c_str());
    }
    params.lambda = 0.5;
    if (!ros::param::get(ns + "/aep/gain/lambda", params.lambda)) {
      ROS_WARN(
          "No lambda specified. Looking for %s. Default is 0.5.",
          (ns + "/aep/gain/lambda").c_str());
    }
    params.extension_range = 1.0;
    if (!ros::param::get(ns + "/aep/tree/extension_range", params.extension_range)) {
      ROS_WARN("No value for maximal extension range specified. Looking for %s. Default is 1.0m.",
          (ns + "/aep/tree/extension_range").c_str());
    }
    params.max_sampling_radius = 10.0;
    if (!ros::param::get(ns + "/aep/tree/max_sampling_radius", params.max_sampling_radius)) {
      ROS_WARN("No value for maximal extension range specified. Looking for %s. Default is 10.0m.",
          (ns + "/aep/tree/max_sampling_radius").c_str());
    }
    params.sigma_thresh = 0.2;
    if (!ros::param::get(ns + "/aep/gain/sigma_thresh", params.sigma_thresh)) {
      ROS_WARN("No value for sigma threshold Looking for %s. Default is 0.2.",
          (ns + "/aep/gain/sigma_thresh").c_str());
    }
    params.init_iterations = 15;
    if (!ros::param::get(ns + "/aep/tree/initial_iterations", params.init_iterations)) {
      ROS_WARN("No number of initial tree iterations specified. Looking for %s. Default is 15.",
          (ns + "/aep/tree/initial_iterations").c_str());
    }
    params.r_max = 4.0;
    if (!ros::param::get(ns + "/aep/gain/r_max", params.r_max)) {
      ROS_WARN("No gain range specified. Looking for %s. Default is 4.0m.",
          (ns + "/aep/gain/r_max").c_str());
    }
    params.r_min = 0.5;
    if (!ros::param::get(ns + "/aep/gain/r_min", params.r_min)) {
      ROS_WARN("No gain range specified. Looking for %s. Default is 0.5m.",
          (ns + "/aep/gain/r_min").c_str());
    }
    if (!ros::param::get(ns + "/boundary/min", params.boundary_min)) {
      ROS_WARN("No x-min value specified. Looking for %s", (ns + "/bbx/minX").c_str());
    }
    if (!ros::param::get(ns + "/boundary/max", params.boundary_max)) {
      ROS_WARN("No x-min value specified. Looking for %s", (ns + "/bbx/minX").c_str());
    }
    params.bounding_radius;
    if (!ros::param::get(ns + "/system/bbx/r", params.bounding_radius)) {
      ROS_WARN("No bounding radius specified. Looking for %s. Default is 0.5m.",
          (ns + "/system/bbx/r").c_str());
    }
    params.cutoff_iterations = 200;
    if (!ros::param::get(ns + "/aep/tree/cutoff_iterations", params.cutoff_iterations)) {
      ROS_WARN("No cutoff iterations value specified. Looking for %s. Default is 200.",
          (ns + "/aep/tree/cutoff_iterations").c_str());
    }
    params.zero_gain = 0.0;
    if (!ros::param::get(ns + "/aep/gain/zero", params.zero_gain)) {
      ROS_WARN("No zero gain value specified. Looking for %s. Default is 0.0.",
          (ns + "/aep/gain/zero").c_str());
    }
    params.d_overshoot_ = 0.5;
    if (!ros::param::get(ns + "/system/bbx/overshoot", params.d_overshoot_)) {
      ROS_WARN(
          "No estimated overshoot value for collision avoidance specified. Looking for %s. Default is 0.5m.",
          (ns + "/system/bbx/overshoot").c_str());
    }
    params.world_frame = "world";
    if (!ros::param::get(ns + "/world_frame", params.robot_frame)) {
      ROS_WARN("No world frame specified. Looking for %s. Default is 'world'.",
          (ns + "/world_frame").c_str());
    }
    params.robot_frame = "base_link";
    if (!ros::param::get(ns + "/robot_frame", params.robot_frame)) {
      ROS_WARN("No robot frame specified. Looking for %s. Default is 'base_link'.",
          (ns + "/robot_frame").c_str());
    }
    params.visualize_tree = false;
    if (!ros::param::get(ns + "/visualize_tree", params.visualize_tree)) {
      ROS_WARN("Looking for %s. Default is false.", (ns + "/visualize_tree").c_str());
    }
    params.visualize_rays = false;
    if (!ros::param::get(ns + "/visualize_rays", params.visualize_rays)) {
      ROS_WARN("Looking for %s. Default is false.", (ns + "/visualize_rays").c_str());
    }
    params.visualize_exploration_area = false;
    if (!ros::param::get(ns + "/visualize_exploration_area", params.visualize_exploration_area)) {
      ROS_WARN("Looking for %s. Default is false.", (ns + "/visualize_exploration_area").c_str());
    }

    return params;
  }
}
