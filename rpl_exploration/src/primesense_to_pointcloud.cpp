#include <pluginlib/class_list_macros.h>
#include <nodelet/nodelet.h>

#include <ros/ros.h>

#include <pcl_ros/point_cloud.h>
#include <pcl_conversions/pcl_conversions.h>

namespace primesense_to_pointcloud {
  class PTPNodelet : public nodelet::Nodelet
  {
  private:
    ros::Subscriber scan_sub_;
    ros::Publisher cloud_pub_;

    int cloud_skip_;
    int cloud_ctr_;

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
      if (!nh_priv.getParam("cloud_skip", cloud_skip_))
      {
        NODELET_ERROR("Failed to get param 'cloud_topic'");
      }
      cloud_ctr_ = 0;

      scan_sub_ = nh.subscribe(scan_topic, 2, &PTPNodelet::scanCallback, this);
      cloud_pub_ = nh.advertise<pcl::PointCloud<pcl::PointXYZ> >(cloud_topic, 1);
    }

    void scanCallback(const pcl::PointCloud<pcl::PointXYZ>::ConstPtr & scan)
    {
      if(cloud_ctr_ == cloud_skip_){
        // pcl::PointCloud<pcl::PointXYZ>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZ>);
        cloud_pub_.publish(scan);
	cloud_ctr_ = 0;
      }
      cloud_ctr_++;
    }
  };
}

PLUGINLIB_EXPORT_CLASS(primesense_to_pointcloud::PTPNodelet, nodelet::Nodelet)
