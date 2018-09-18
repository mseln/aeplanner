#include <pluginlib/class_list_macros.h>
#include <nodelet/nodelet.h>

#include <ros/ros.h>

#include <sensor_msgs/LaserScan.h>
#include <pcl_ros/point_cloud.h>
#include <pcl_conversions/pcl_conversions.h>

#include <limits>

namespace laserscan_to_pointcloud {
  class LTPNodelet : public nodelet::Nodelet
  {
  private:
    ros::Subscriber scan_sub_;

    ros::Publisher cloud_pub_;

  private:
    virtual void onInit()
    {
      ros::NodeHandle & nh = getNodeHandle();
      ros::NodeHandle & nh_priv = getPrivateNodeHandle();

      std::string scan_topic;
      if (!nh_priv.getParam("scan_topic", scan_topic))
      {
        NODELET_ERROR("Failed to get param 'scan_topic'");
      }
      std::string cloud_topic;
      if (!nh_priv.getParam("cloud_topic", cloud_topic))
      {
        NODELET_ERROR("Failed to get param 'cloud_topic'");
      }

      scan_sub_ = nh.subscribe(scan_topic, 2, &LTPNodelet::scanCallback, this);
      cloud_pub_ = nh.advertise<pcl::PointCloud<pcl::PointXYZ> >(cloud_topic, 1);
    }

    void scanCallback(const sensor_msgs::LaserScan::ConstPtr & scan)
    {
      pcl::PointCloud<pcl::PointXYZ>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZ>);
      cloud->header.frame_id = scan->header.frame_id;
      cloud->header.stamp = pcl_conversions::toPCL(scan->header.stamp);

      cloud->is_dense = false;
      cloud->reserve(scan->ranges.size());

      double current_angle = scan->angle_min - scan->angle_increment;
      for (size_t i = 0; i < scan->ranges.size(); ++i)
      {
        current_angle += scan->angle_increment;

        pcl::PointXYZ point;

        if (scan->ranges[i] < scan->range_min)
        {
          // Invalid range
          point.x = std::numeric_limits<float>::quiet_NaN();
          point.y = std::numeric_limits<float>::quiet_NaN();
          point.z = std::numeric_limits<float>::quiet_NaN();
          cloud->is_dense = false;
        }
        else if (scan->ranges[i] > scan->range_max)
        {
          point.x = (scan->range_max+0.001) * std::cos(current_angle); // + 1 mm to make it a bit longer than max length
          point.y = (scan->range_max+0.001) * std::sin(current_angle); // so entire scan will be classified as free
          point.z = 0;
        }
        else
        {
          point.x = scan->ranges[i] * std::cos(current_angle);
          point.y = scan->ranges[i] * std::sin(current_angle);
          point.z = 0;
        }

        cloud->points.push_back(point);
      }

      cloud_pub_.publish(cloud);
    }
  };
}

PLUGINLIB_EXPORT_CLASS(laserscan_to_pointcloud::LTPNodelet, nodelet::Nodelet)
