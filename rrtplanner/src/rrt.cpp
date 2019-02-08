#include <rrtplanner/rrt.h>

namespace aeplanner_ns
{
Rrt::Rrt(const ros::NodeHandle& nh)
  : nh_(nh)
  , frame_id_("map")
  , path_pub_(nh_.advertise<visualization_msgs::Marker>("rrt_path", 1000))
  , octomap_sub_(nh_.subscribe("octomap", 1, &Rrt::octomapCallback, this))
  , as_(nh_, "rrt", boost::bind(&Rrt::execute, this, _1), false)
{
  std::string ns = ros::this_node::getNamespace();
  bounding_radius_ = 0.5;
  bounding_overshoot_ = 0.5;
  extension_range_ = 1.0;
  min_nodes_ = 100;
  if (!ros::param::get(ns + "/rrt/min_nodes", min_nodes_))
    ROS_WARN("No minimum nodes specified default is 100");
  if (!ros::param::get(ns + "/system/bbx/r", bounding_radius_))
    ROS_WARN("No bounding radius specified default is 0.5 m");
  if (!ros::param::get(ns + "/system/bbx/overshoot", bounding_overshoot_))
    ROS_WARN("No overshoot paramerer specified, default is 0.5 m");
  if (!ros::param::get(ns + "/aep/tree/extension_range", extension_range_))
    ROS_WARN("No extension range specified, default is 1.0 m");
  if (!ros::param::get(ns + "/boundary/min", boundary_min_))
    ROS_WARN("No boundary min specified.");
  if (!ros::param::get(ns + "/boundary/max", boundary_max_))
    ROS_WARN("No boundary max specified.");

  ot_ = std::make_shared<octomap::OcTree>(
      1);  // Create dummy OcTree to prevent crash due to ot_ tree not initialized
  as_.start();
}

void Rrt::execute(const rrtplanner::rrtGoalConstPtr& goal)
{
  rrtplanner::rrtResult result;
  if (!ot_)
  {
    ROS_WARN("No octomap received");
    as_.setSucceeded(result);
    return;
  }
  if (!goal->goal_poses.poses.size())
  {
    ROS_WARN("No goals received");
    as_.setSucceeded(result);
    return;
  }

  int N = min_nodes_;
  double l = extension_range_;
  double r = bounding_radius_;
  double r_os = bounding_overshoot_;
  std::vector<RrtNode*> found_goals;

  kdtree* kd_tree = kd_create(3);    // Initalize tree
  kdtree* goal_tree = kd_create(3);  // kd tree with all goals
  for (int i = 0; i < goal->goal_poses.poses.size(); ++i)
  {
    Eigen::Vector3d* g = new Eigen::Vector3d(goal->goal_poses.poses[i].position.x,
                                             goal->goal_poses.poses[i].position.y,
                                             goal->goal_poses.poses[i].position.z);
    kd_insert3(goal_tree, (*g)[0], (*g)[1], (*g)[2], g);
  }

  // Initialize root position
  RrtNode* root = new RrtNode;
  root->pos[0] = goal->start.pose.position.x;
  root->pos[1] = goal->start.pose.position.y;
  root->pos[2] = goal->start.pose.position.z;
  root->parent = NULL;

  kd_insert3(kd_tree, root->pos[0], root->pos[1], root->pos[2], root);

  visualizeNode(goal->start.pose.position, 1000);
  visualizeGoals(goal->goal_poses.poses);

  for (int i = 0; i < N /* or !found_goals.size() */; ++i)
  {
    // Sample new position
    Eigen::Vector3d z_samp = sample();

    // Get nearest neighbour
    RrtNode* z_parent = chooseParent(kd_tree, z_samp, l);
    if (!z_parent)
      continue;

    // Calculate position for new node
    Eigen::Vector3d new_pos = getNewPos(z_samp, z_parent->pos, l);
    Eigen::Vector3d direction = new_pos - z_parent->pos;

    if (!collisionLine(z_parent->pos, new_pos + direction.normalized() * r_os, r))
    {
      // Add node to tree
      RrtNode* z_new = addNodeToTree(kd_tree, z_parent, new_pos);
      rewire(kd_tree, z_new, l, r, r_os);

      visualizeEdge(z_new, i);

      // Check if goal has been reached
      RrtNode* tmp_goal = getGoal(goal_tree, z_new, l, r, r_os);
      if (tmp_goal)
      {
        found_goals.push_back(tmp_goal);
      }
    }
    else
    {
      --i;
    }
  }

  result.path = getBestPath(found_goals);

  delete root;
  kd_free(kd_tree);
  kd_free(goal_tree);

  as_.setSucceeded(result);
}

void Rrt::octomapCallback(const octomap_msgs::Octomap& msg)
{
  octomap::AbstractOcTree* aot = octomap_msgs::msgToMap(msg);
  octomap::OcTree* ot = (octomap::OcTree*)aot;
  ot_ = std::make_shared<octomap::OcTree>(*ot);

  if (ot)
    delete ot;
}

Eigen::Vector3d Rrt::sample()
{
  Eigen::Vector3d x_samp;
  for (int i = 0; i < 3; ++i)
  {
    do
    {
      x_samp[i] = 60 * (((double)rand()) / ((double)RAND_MAX) - 0.5);
    } while(x_samp[i] < boundary_min_[i] or x_samp[i] > boundary_max_[i]);
  }

  return x_samp;
}

RrtNode* Rrt::chooseParent(kdtree* kd_tree, Eigen::Vector3d node, double l)
{
  kdres* nearest = kd_nearest_range3(kd_tree, node[0], node[1], node[2], l + 0.5);
  if (kd_res_size(nearest) <= 0)
  {
    nearest = kd_nearest3(kd_tree, node[0], node[1], node[2]);
  }
  if (kd_res_size(nearest) <= 0)
  {
    kd_res_free(nearest);
    return NULL;
  }

  RrtNode* node_nn = (RrtNode*)kd_res_item_data(nearest);
  int i = 0;

  RrtNode* best_node = node_nn;
  while (!kd_res_end(nearest))
  {
    node_nn = (RrtNode*)kd_res_item_data(nearest);
    if (best_node and node_nn->cost() < best_node->cost())
      best_node = node_nn;

    kd_res_next(nearest);
  }

  kd_res_free(nearest);
  return best_node;
}

void Rrt::rewire(kdtree* kd_tree, RrtNode* new_node, double l, double r, double r_os)
{
  RrtNode* node_nn;
  kdres* nearest = kd_nearest_range3(kd_tree, new_node->pos[0], new_node->pos[1],
                                     new_node->pos[2], l + 0.5);
  while (!kd_res_end(nearest))
  {
    node_nn = (RrtNode*)kd_res_item_data(nearest);
    if (node_nn->cost() > new_node->cost() + (node_nn->pos - new_node->pos).norm())
    {
      if (!collisionLine(
              new_node->pos,
              node_nn->pos + (node_nn->pos - new_node->pos).normalized() * r_os, r))
        node_nn->parent = new_node;
    }
    kd_res_next(nearest);
  }
}

Eigen::Vector3d Rrt::getNewPos(Eigen::Vector3d sampled, Eigen::Vector3d parent,
                               double l)
{
  Eigen::Vector3d direction = sampled - parent;
  if (direction.norm() > l)
    direction = l * direction.normalized();

  return parent + direction;
}

RrtNode* Rrt::addNodeToTree(kdtree* kd_tree, RrtNode* parent,
                            Eigen::Vector3d new_pos)
{
  RrtNode* new_node = new RrtNode;
  new_node->pos = new_pos;

  new_node->parent = parent;
  parent->children.push_back(new_node);
  kd_insert3(kd_tree, new_node->pos[0], new_node->pos[1], new_node->pos[2],
             new_node);

  return new_node;
}

RrtNode* Rrt::getGoal(kdtree* goal_tree, RrtNode* new_node, double l, double r,
                      double r_os)
{
  kdres* nearest_goal =
      kd_nearest3(goal_tree, new_node->pos[0], new_node->pos[1], new_node->pos[2]);
  if (kd_res_size(nearest_goal) <= 0)
  {
    kd_res_free(nearest_goal);
    return NULL;
  }
  Eigen::Vector3d* g_nn = (Eigen::Vector3d*)kd_res_item_data(nearest_goal);
  kd_res_free(nearest_goal);

  if ((*g_nn - new_node->pos).norm() < 1.5)
    if (!collisionLine(new_node->pos,
                       *g_nn + (*g_nn - new_node->pos).normalized() * r_os, r))
      return new_node;

  return NULL;
}

nav_msgs::Path Rrt::getBestPath(std::vector<RrtNode*> goals)
{
  nav_msgs::Path path;
  if (goals.size() == 0)
  {
    return path;
  }

  RrtNode* best_node = goals[0];

  for (int i = 0; i < goals.size(); ++i)
    if (best_node->cost() > goals[i]->cost())
      best_node = goals[i];

  RrtNode* n = best_node;
  for (int id = 0; n->parent; ++id)
  {
    geometry_msgs::PoseStamped p;
    p.pose.position.x = n->pos[0];
    p.pose.position.y = n->pos[1];
    p.pose.position.z = n->pos[2];
    Eigen::Quaternion<double> q;
    Eigen::Vector3d init(1.0, 0.0, 0.0);
    // Zero out rotation along
    // x and y axis so only
    // yaw is kept
    Eigen::Vector3d dir(n->pos[0] - n->parent->pos[0], n->pos[1] - n->parent->pos[1],
                        0);
    q.setFromTwoVectors(init, dir);

    p.pose.orientation.x = q.x();
    p.pose.orientation.y = q.y();
    p.pose.orientation.z = q.z();
    p.pose.orientation.w = q.w();

    path.poses.push_back(p);
    visualizePose(p.pose, id);

    n = n->parent;
  }

  visualizePath(best_node);
  return path;
}

std::vector<geometry_msgs::Pose> Rrt::checkIfGoalReached(kdtree* goal_tree,
                                                         RrtNode* new_node, double l,
                                                         double r, double r_os)
{
  std::vector<geometry_msgs::Pose> path;

  kdres* nearest_goal =
      kd_nearest3(goal_tree, new_node->pos[0], new_node->pos[1], new_node->pos[2]);
  if (kd_res_size(nearest_goal) <= 0)
  {
    kd_res_free(nearest_goal);
    return path;
  }
  Eigen::Vector3d* g_nn = (Eigen::Vector3d*)kd_res_item_data(nearest_goal);
  kd_res_free(nearest_goal);

  if ((*g_nn - new_node->pos).norm() < 2 * l)
  {
    if (!collisionLine(new_node->pos,
                       *g_nn + (*g_nn - new_node->pos).normalized() * r_os, r))
    {
      RrtNode* n = new_node;
      for (int id = 0; n->parent; ++id)
      {
        geometry_msgs::Pose p;
        p.position.x = n->pos[0];
        p.position.y = n->pos[1];
        p.position.z = n->pos[2];
        Eigen::Quaternion<double> q;
        Eigen::Vector3d init(1.0, 0.0, 0.0);
        // Zero out rotation
        // along x and y axis
        // so only yaw is kept
        Eigen::Vector3d dir(n->pos[0] - n->parent->pos[0],
                            n->pos[1] - n->parent->pos[1], 0);
        q.setFromTwoVectors(init, dir);

        p.orientation.x = q.x();
        p.orientation.y = q.y();
        p.orientation.z = q.z();
        p.orientation.w = q.w();

        path.push_back(p);
        visualizePose(p, id);

        n = n->parent;
      }

      visualizePath(new_node);
    }
  }

  return path;
}

bool Rrt::collisionLine(Eigen::Vector3d p1, Eigen::Vector3d p2, double r)
{
  std::shared_ptr<octomap::OcTree> ot = ot_;

  octomap::point3d start(p1[0], p1[1], p1[2]);
  octomap::point3d end(p2[0], p2[1], p2[2]);
  octomap::point3d min(std::min(p1[0], p2[0]) - r, std::min(p1[1], p2[1]) - r,
                       std::min(p1[2], p2[2]) - r);
  octomap::point3d max(std::max(p1[0], p2[0]) + r, std::max(p1[1], p2[1]) + r,
                       std::max(p1[2], p2[2]) + r);
  double lsq = (end - start).norm_sq();
  double rsq = r * r;

  octomap::point3d query(p2[0], p2[1], p2[2]);
  octomap::OcTreeNode* ot_res = ot->search(query);
  if (!ot_res)
    return true;

  for (octomap::OcTree::leaf_bbx_iterator it = ot->begin_leafs_bbx(min, max),
                                          it_end = ot->end_leafs_bbx();
       it != it_end; ++it)
  {
    octomap::point3d pt(it.getX(), it.getY(), it.getZ());

    if (it->getLogOdds() > 0)
    {  // Node is occupied
      if (CylTest_CapsFirst(start, end, lsq, rsq, pt) > 0 or (end - pt).norm() < r)
      {
        return true;
      }
    }
  }

  // Check all unknown nodes
  /*
    octomap::point3d_list
    node_centers;
    ot_->getUnknownLeafCenters(node_centers,
    min, max);
    std::list<octomath::Vector3>::iterator
    it; for (it =
    node_centers.begin(); it
    != node_centers.end();
    ++it){
      if(CylTest_CapsFirst(start,
    end, lsq, rsq, *it) > 0 or
    (end-*it).norm() < r) {
        return true;
      }
    }
    */

  return false;
}

//-----------------------------------------------------------------------------
// Name: CylTest_CapsFirst
// Orig: Greg James -
// gjames@NVIDIA.com Lisc:
// Free code - no warranty &
// no money back.  Use it all
// you want Desc:
//    This function tests if
//    the 3D point 'pt' lies
//    within an arbitrarily
// oriented cylinder.  The
// cylinder is defined by an
// axis from 'pt1' to 'pt2',
// the axis having a length
// squared of 'lsq'
// (pre-compute for each
// cylinder to avoid repeated
// work!), and radius squared
// of 'rsq'.
//    The function tests
//    against the end caps
//    first, which is cheap ->
//    only
// a single dot product to
// test against the parallel
// cylinder caps.  If the
// point is within these, more
// work is done to find the
// distance of the point from
// the cylinder axis.
//    Fancy Math (TM) makes
//    the whole test possible
//    with only two
//    dot-products
// a subtract, and two
// multiplies.  For clarity,
// the 2nd mult is kept as a
// divide.  It might be faster
// to change this to a mult by
// also passing in 1/lengthsq
// and using that instead.
//    Elminiate the first 3
//    subtracts by specifying
//    the cylinder as a base
// point on one end cap and a
// vector to the other end cap
// (pass in {dx,dy,dz} instead
// of 'pt2' ).
//
//    The dot product is
//    constant along a plane
//    perpendicular to a
//    vector. The magnitude of
//    the cross product
//    divided by one vector
//    length is
// constant along a cylinder
// surface defined by the
// other vector as axis.
//
// Return:  -1.0 if point is
// outside the cylinder
// Return:  distance squared
// from cylinder axis if point
// is inside.
//
//-----------------------------------------------------------------------------
float CylTest_CapsFirst(const octomap::point3d& pt1, const octomap::point3d& pt2,
                        float lsq, float rsq, const octomap::point3d& pt)
{
  float dx, dy,
      dz;  // vector d  from
           // line segment
           // point 1 to point
           // 2
  float pdx, pdy,
      pdz;  // vector pd from
            // point 1 to test
            // point
  float dot, dsq;

  dx = pt2.x() - pt1.x();  // translate
                           // so pt1 is
                           // origin.
                           // Make vector
                           // from
  dy = pt2.y() - pt1.y();  // pt1 to
                           // pt2.  Need
                           // for this
                           // is easily
                           // eliminated
  dz = pt2.z() - pt1.z();

  pdx = pt.x() - pt1.x();  // vector from
                           // pt1 to test
                           // point.
  pdy = pt.y() - pt1.y();
  pdz = pt.z() - pt1.z();

  // Dot the d and pd vectors
  // to see if point lies
  // behind the cylinder cap
  // at pt1.x, pt1.y, pt1.z

  dot = pdx * dx + pdy * dy + pdz * dz;

  // If dot is less than zero
  // the point is behind the
  // pt1 cap. If greater than
  // the cylinder axis line
  // segment length squared
  // then the point is outside
  // the other end cap at pt2.

  if (dot < 0.0f || dot > lsq)
  {
    return (-1.0f);
  }
  else
  {
    // Point lies within the
    // parallel caps, so find
    // distance squared from
    // point to line, using
    // the fact that sin^2 +
    // cos^2 = 1 the dot =
    // cos() * |d||pd|, and
    // cross*cross = sin^2 *
    // |d|^2 * |pd|^2
    // Carefull: '*' means
    // mult for scalars and
    // dotproduct for vectors
    // In short, where dist is
    // pt distance to cyl
    // axis: dist = sin( pd to
    // d ) * |pd| distsq = dsq
    // = (1 - cos^2( pd to d))
    // * |pd|^2 dsq = ( 1 -
    // (pd * d)^2 / (|pd|^2 *
    // |d|^2) ) * |pd|^2 dsq =
    // pd * pd - dot * dot /
    // lengthsq
    //  where lengthsq is d*d
    //  or |d|^2 that is
    //  passed into this
    //  function

    // distance squared to the
    // cylinder axis:

    dsq = (pdx * pdx + pdy * pdy + pdz * pdz) - dot * dot / lsq;

    if (dsq > rsq)
    {
      return (-1.0f);
    }
    else
    {
      return (dsq);  // return
                     // distance
                     // squared
                     // to
                     // axis
    }
  }
}

void Rrt::visualizeNode(geometry_msgs::Point pos, int id)
{
  visualization_msgs::Marker a;
  a.header.stamp = ros::Time::now();
  a.header.seq = id;
  a.header.frame_id = frame_id_;
  a.id = id;
  a.ns = "nodes";
  a.type = visualization_msgs::Marker::SPHERE;
  a.action = visualization_msgs::Marker::ADD;
  a.pose.position = pos;

  a.scale.x = 0.2;
  a.scale.y = 0.2;
  a.scale.z = 0.2;
  a.color.r = 0.2;
  a.color.g = 0.7;
  a.color.b = 0.2;
  ;
  a.color.a = 1;
  a.lifetime = ros::Duration(5.0);
  a.frame_locked = false;
  path_pub_.publish(a);
}

void Rrt::visualizePose(geometry_msgs::Pose pose, int id)
{
  visualization_msgs::Marker a;
  a.header.stamp = ros::Time::now();
  a.header.seq = id;
  a.header.frame_id = frame_id_;
  a.id = id;
  a.ns = "pose";
  a.type = visualization_msgs::Marker::ARROW;
  a.action = visualization_msgs::Marker::ADD;
  a.pose = pose;
  a.scale.x = 0.4;
  a.scale.y = 0.1;
  a.scale.z = 0.1;
  a.color.r = 1.0;
  a.color.g = 0.0;
  a.color.b = 0.0;
  a.color.a = 1.0;
  a.lifetime = ros::Duration(5.0);
  a.frame_locked = false;

  path_pub_.publish(a);
}

void Rrt::visualizeEdge(RrtNode* node, int id)
{
  visualization_msgs::Marker a;
  a.header.stamp = ros::Time::now();
  a.header.seq = id;
  a.header.frame_id = frame_id_;
  a.id = id;
  a.ns = "vp_branches";
  a.type = visualization_msgs::Marker::ARROW;
  a.action = visualization_msgs::Marker::ADD;
  a.pose.position.x = node->parent->pos[0];
  a.pose.position.y = node->parent->pos[1];
  a.pose.position.z = node->parent->pos[2];
  Eigen::Quaternion<double> q;
  Eigen::Vector3d init(1.0, 0.0, 0.0);
  Eigen::Vector3d dir(node->pos[0] - node->parent->pos[0],
                      node->pos[1] - node->parent->pos[1],
                      node->pos[2] - node->parent->pos[2]);
  q.setFromTwoVectors(init, dir);
  q.normalize();
  a.pose.orientation.x = q.x();
  a.pose.orientation.y = q.y();
  a.pose.orientation.z = q.z();
  a.pose.orientation.w = q.w();
  a.scale.x = dir.norm();
  a.scale.y = 0.05;
  a.scale.z = 0.05;
  a.color.r = 1.0;
  a.color.g = 0.3;
  a.color.b = 0.7;
  a.color.a = 1.0;
  a.lifetime = ros::Duration(5.0);
  a.frame_locked = false;

  path_pub_.publish(a);
}

void Rrt::visualizePath(RrtNode* node)
{
  for (int id = 0; node->parent; ++id)
  {
    visualization_msgs::Marker a;
    a.header.stamp = ros::Time::now();
    a.header.seq = id;
    a.header.frame_id = frame_id_;
    a.id = id;
    a.ns = "path";
    a.type = visualization_msgs::Marker::ARROW;
    a.action = visualization_msgs::Marker::ADD;
    a.pose.position.x = node->parent->pos[0];
    a.pose.position.y = node->parent->pos[1];
    a.pose.position.z = node->parent->pos[2];
    Eigen::Quaternion<double> q;
    Eigen::Vector3d init(1.0, 0.0, 0.0);
    Eigen::Vector3d dir(node->pos[0] - node->parent->pos[0],
                        node->pos[1] - node->parent->pos[1],
                        node->pos[2] - node->parent->pos[2]);
    q.setFromTwoVectors(init, dir);
    q.normalize();
    a.pose.orientation.x = q.x();
    a.pose.orientation.y = q.y();
    a.pose.orientation.z = q.z();
    a.pose.orientation.w = q.w();
    a.scale.x = dir.norm();
    a.scale.y = 0.07;
    a.scale.z = 0.07;
    a.color.r = 0.7;
    a.color.g = 0.7;
    a.color.b = 0.3;
    a.color.a = 1.0;
    a.lifetime = ros::Duration(100.0);
    a.frame_locked = false;

    path_pub_.publish(a);

    node = node->parent;
  }
}

void Rrt::visualizeGoals(std::vector<geometry_msgs::Pose> goals)
{
  for (int i = 0; i < goals.size(); ++i)
  {
    visualization_msgs::Marker a;
    a.header.stamp = ros::Time::now();
    a.header.seq = i;
    a.header.frame_id = frame_id_;
    a.id = i;
    a.ns = "goals";
    a.type = visualization_msgs::Marker::ARROW;
    a.action = visualization_msgs::Marker::ADD;
    a.pose = goals[i];

    a.scale.x = 0.2;
    a.scale.y = 0.1;
    a.scale.z = 0.1;
    a.color.r = 1.0;
    a.color.g = 0.3;
    a.color.b = 0.7;
    a.color.a = 1;
    a.lifetime = ros::Duration(100.0);
    a.frame_locked = false;
    path_pub_.publish(a);
  }
}

}  // namespace aeplanner_ns
