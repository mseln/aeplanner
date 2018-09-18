#ifndef READ_PARAMS_H
#define READ_PARAMS_H

namespace aeplanner
{
  struct Params
  {
    double hfov;
    double vfov;
    double r_max;
    double r_min;

    double dr;
    double dphi;
    double dtheta;

    double lambda;
    double zero_gain;
    double extension_range;
    double max_sampling_radius;
    double sigma_thresh;

    double d_overshoot_;
    double bounding_radius;

    int init_iterations;
    int cutoff_iterations;

    std::vector<double> boundary_min;
    std::vector<double> boundary_max;

    std::string robot_frame;
    std::string world_frame;

    bool visualize_tree;
    bool visualize_rays;
    bool visualize_exploration_area;
  };

  Params readParams();
}

#endif
