#include <ros/ros.h>
#include <aeplanner_evaluation/Coverage.h>

#include <octomap/octomap.h>
#include <octomap_msgs/conversions.h>

void octomapCallback(const octomap_msgs::Octomap& msg);

class CoverageEvaluator{
  private:
    ros::NodeHandle nh_;
    ros::Subscriber octomap_sub_;
    ros::ServiceServer coverage_srv_;

    double total_volume_;
    double coverage_, free_, occupied_, unmapped_;
  public:
    CoverageEvaluator(ros::NodeHandle& nh) : nh_(nh),
                                             octomap_sub_(nh_.subscribe("octomap", 10, &CoverageEvaluator::octomapCallback2, this)),
                                             coverage_srv_(nh.advertiseService("get_coverage", &CoverageEvaluator::coverageSrvCallback, this)){
      if (!nh.getParam("volume", total_volume_)) {
        ROS_ERROR("No volume found...");
      }
    }

    void octomapCallback(const octomap_msgs::Octomap& msg);
    void octomapCallback2(const octomap_msgs::Octomap& msg);
    bool coverageSrvCallback(aeplanner_evaluation::Coverage::Request& request, aeplanner_evaluation::Coverage::Response& response);

};

int main(int argc, char** argv) {
  ros::init(argc, argv, "coverage_evaluation");
  ros::NodeHandle nh;

  CoverageEvaluator ce_(nh);

  ros::spin();
}

void CoverageEvaluator::octomapCallback(const octomap_msgs::Octomap& msg){
  octomap::AbstractOcTree* aot = octomap_msgs::msgToMap(msg);
  octomap::OcTree * ot = (octomap::OcTree*)aot;
  double volume = 0;

  for(octomap::OcTree::leaf_iterator it  = ot->begin_leafs();
                                     it != ot->end_leafs(); ++it){
    // if(!ot->isNodeOccupied(*it)){
      double l = it.getSize();
      volume += l*l*l;
    // }
  }

  coverage_ = volume;
}

void CoverageEvaluator::octomapCallback2(const octomap_msgs::Octomap& msg){
  octomap::AbstractOcTree* aot = octomap_msgs::msgToMap(msg);
  octomap::OcTree * ot = (octomap::OcTree*)aot;
  
  double occupied = 0;
  double free     = 0;
  double unmapped = 0;

  double min[3] = {-10 , -15, 0};
  double max[3] = { 10,  15, 3};
  double res = 0.2;
  double dV = res*res*res;

  for(double x = min[0]; x < max[0]-res/2; x+=res){
    for(double y = min[1]; y < max[1]-res/2; y+=res){
      for(double z = min[2]; z < max[2]-res/2; z+=res){
        octomap::OcTreeNode* result = ot->search(x+res/2,y+res/2,z+res/2);
        if (!result) 
          unmapped+=dV;
        else if(result->getLogOdds() > 0)
          occupied+=dV;
        else
          free+=dV;
      }
    }
  }

  coverage_ = free+occupied;
  free_ = free;
  occupied_ = occupied;
  unmapped_ = unmapped;
}

bool CoverageEvaluator::coverageSrvCallback(aeplanner_evaluation::Coverage::Request& request, aeplanner_evaluation::Coverage::Response& response){
  response.coverage = coverage_;
  response.free     = free_;
  response.occupied = occupied_;
  response.unmapped = unmapped_;
  return true;
}
