#include <fstream>

#include <ros/package.h>
#include <ros/ros.h>
#include <tf/tf.h>

#include <aeplanner_evaluation/Coverage.h>
#include <std_srvs/Empty.h>

#include <actionlib/client/simple_action_client.h>
#include <aeplanner/Node.h>
#include <aeplanner/aeplannerAction.h>
#include <rpl_exploration/FlyToAction.h>
#include <rrtplanner/rrtAction.h>

#include <geometry_msgs/PoseStamped.h>

#include <nav_msgs/Path.h>
#include <tf2/utils.h>

int main(int argc, char** argv)
{
  ros::init(argc, argv, "exploration");
  ros::NodeHandle nh;
  ROS_INFO("Started exploration");

  // Open logfile;
  std::string path = ros::package::getPath("rpl_exploration");
  std::ofstream logfile, pathfile;
  logfile.open(path + "/data/logfile.csv");
  pathfile.open(path + "/data/path.csv");

  ros::Publisher pub(nh.advertise<geometry_msgs::PoseStamped>("/mavros/"
                                                              "setpoint_position/"
                                                              "local",
                                                              1000));

  ros::ServiceClient coverage_srv =
      nh.serviceClient<aeplanner_evaluation::Coverage>("/get_coverage");

  // wait for fly_to server to start
  // ROS_INFO("Waiting for fly_to action server");
  actionlib::SimpleActionClient<rpl_exploration::FlyToAction> ac("fly_to", true);
  // ac.waitForServer(); //will wait for infinite time
  // ROS_INFO("Fly to ction server started!");

  // wait for aep server to start
  ROS_INFO("Waiting for aeplanner action server");
  actionlib::SimpleActionClient<aeplanner::aeplannerAction> aep_ac("make_plan",
                                                                   true);
  aep_ac.waitForServer();  // will wait for infinite time
  ROS_INFO("aeplanner action server started!");

  // wait for fly_to server to start
  ROS_INFO("Waiting for rrt action server");
  actionlib::SimpleActionClient<rrtplanner::rrtAction> rrt_ac("rrt", true);
  // rrt_ac.waitForServer(); //will wait for infinite time
  ROS_INFO("rrt Action server started!");

  // Get current pose
  geometry_msgs::PoseStamped::ConstPtr init_pose =
      ros::topic::waitForMessage<geometry_msgs::PoseStamped>("/mavros/"
                                                             "local_position/pose");
  double init_yaw = tf2::getYaw(init_pose->pose.orientation);
  // Up 2 meters and then forward one meter
  double initial_positions[8][4] = {
    { init_pose->pose.position.x, init_pose->pose.position.y,
      init_pose->pose.position.z + 2.0, init_yaw },
    { init_pose->pose.position.x + 1.0 * std::cos(init_yaw),
      init_pose->pose.position.y + 1.0 * std::sin(init_yaw),
      init_pose->pose.position.z + 2.0, init_yaw },
  };

  // This is the initialization motion, necessary that the known free space
  // allows the planning of initial paths.
  ROS_INFO("Starting the planner: Performing initialization motion");
  geometry_msgs::PoseStamped last_pose;

  for (int i = 0; i < 2; ++i)
  {
    rpl_exploration::FlyToGoal goal;
    goal.pose.pose.position.x = initial_positions[i][0];
    goal.pose.pose.position.y = initial_positions[i][1];
    goal.pose.pose.position.z = initial_positions[i][2];
    goal.pose.pose.orientation =
        tf::createQuaternionMsgFromYaw(initial_positions[i][3]);
    last_pose.pose = goal.pose.pose;

    ROS_INFO_STREAM("Sending initial goal...");
    ac.sendGoal(goal);

    ac.waitForResult(ros::Duration(0));
  }

  // Start planning: The planner is called and the computed path sent to the
  // controller.
  int iteration = 0;
  int actions_taken = 1;

  ros::Time start = ros::Time::now();
  while (ros::ok())
  {
    ROS_INFO_STREAM("Planning iteration " << iteration);
    aeplanner::aeplannerGoal aep_goal;
    aep_goal.header.stamp = ros::Time::now();
    aep_goal.header.seq = iteration;
    aep_goal.header.frame_id = "map";
    aep_goal.actions_taken = actions_taken;
    aep_ac.sendGoal(aep_goal);

    while (!aep_ac.waitForResult(ros::Duration(0.05)))
    {
      pub.publish(last_pose);
    }

    ros::Duration fly_time;
    if (aep_ac.getResult()->is_clear)
    {
      actions_taken = 0;

      ros::Time s = ros::Time::now();
      geometry_msgs::PoseStamped goal_pose = aep_ac.getResult()->pose;
      // Write path to file
      pathfile << goal_pose.pose.position.x << ", " << goal_pose.pose.position.y
               << ", " << goal_pose.pose.position.z << ", n" << std::endl;

      last_pose.pose = goal_pose.pose;
      rpl_exploration::FlyToGoal goal;
      goal.pose = goal_pose;
      ac.sendGoal(goal);

      ac.waitForResult(ros::Duration(0));

      fly_time = ros::Time::now() - s;
    }
    else
    {
      rrtplanner::rrtGoal rrt_goal;
      rrt_goal.start.header.stamp = ros::Time::now();
      rrt_goal.start.header.frame_id = "map";
      rrt_goal.start.pose = last_pose.pose;
      if (!aep_ac.getResult()->frontiers.poses.size())
      {
        ROS_WARN("Exploration complete!");
        break;
      }
      for (auto it = aep_ac.getResult()->frontiers.poses.begin();
           it != aep_ac.getResult()->frontiers.poses.end(); ++it)
      {
        rrt_goal.goal_poses.poses.push_back(*it);
      }

      rrt_ac.sendGoal(rrt_goal);
      while (!rrt_ac.waitForResult(ros::Duration(0.05)))
      {
        pub.publish(last_pose);
      }
      nav_msgs::Path path = rrt_ac.getResult()->path;

      ros::Time s = ros::Time::now();
      for (int i = path.poses.size() - 1; i >= 0; --i)
      {
        geometry_msgs::Pose goal_pose = path.poses[i].pose;
        // Write path to file
        pathfile << goal_pose.position.x << ", " << goal_pose.position.y << ", "
                 << goal_pose.position.z << ", f" << std::endl;

        last_pose.pose = goal_pose;
        rpl_exploration::FlyToGoal goal;
        goal.pose.pose = goal_pose;
        ac.sendGoal(goal);

        ac.waitForResult(ros::Duration(0));
      }
      actions_taken = -1;
      fly_time = ros::Time::now() - s;
    }

    ros::Duration elapsed = ros::Time::now() - start;

    ROS_INFO_STREAM("Iteration: "       << iteration << "  " <<
                    "Time: "            << elapsed << "  " <<
                    "Sampling: "        << aep_ac.getResult()->sampling_time.data << "  " <<
                    "Planning: "        << aep_ac.getResult()->planning_time.data << "  " <<
                    "Collision check: " << aep_ac.getResult()->collision_check_time.data  << "  " <<
                    "Flying: "          << fly_time << " " <<
                    "Tree size: "       << aep_ac.getResult()->tree_size);

    logfile << iteration << ", " 
            << elapsed << ", "
            << aep_ac.getResult()->sampling_time.data << ", "
            << aep_ac.getResult()->planning_time.data << ", "
            << aep_ac.getResult()->collision_check_time.data << ", "
            << fly_time << ", "
            << aep_ac.getResult()->tree_size << std::endl;

    iteration++;
  }

  pathfile.close();
  logfile.close();
}
