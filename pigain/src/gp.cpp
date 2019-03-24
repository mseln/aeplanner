#include <pigain/gp.h>

#include <eigen3/Eigen/Cholesky>

#include <omp.h>

#include <ros/ros.h>

namespace pig
{
Eigen::MatrixXd sqExpKernel(const Eigen::Matrix<double, Eigen::Dynamic, 3>& x1,
                            const Eigen::Matrix<double, Eigen::Dynamic, 3>& x2, double hyp_l, double hyp_sigma_f,
                            double hyp_sigma_n)
{
  size_t n1 = x1.rows();
  size_t n2 = x2.rows();
  Eigen::MatrixXd k(n1, n2);

  // #pragma omp parallel for
  for (size_t i = 0; i < n2; ++i)
  {
    // Eigen::Matrix<double, Eigen::Dynamic, 3> temp = x1 - x2.row(i);

    // FIXME: Optimize (find cool Eigen functions)
    for (size_t j = 0; j < n1; ++j)
    {
      // double l = temp.row(j).norm();
      double l = (x1.row(j) - x2.row(i)).norm();

      k(j, i) = std::pow(hyp_sigma_f, 2) * std::exp(-0.5 * std::pow(l / hyp_l, 2));
    }
  }

  return k;
}

std::pair<double, double> gp(const Eigen::VectorXd& y, const Eigen::Matrix<double, Eigen::Dynamic, 3>& x,
                             const Eigen::Matrix<double, Eigen::Dynamic, 3>& x_star, double hyp_l, double hyp_sigma_f,
                             double hyp_sigma_n)
{
  if (y.size() == 0 || x.size() == 0)
  {
    return std::make_pair(0, 0);
  }

  Eigen::MatrixXd k = sqExpKernel(x, x, hyp_l, hyp_sigma_f, hyp_sigma_n);
  Eigen::MatrixXd k_star = sqExpKernel(x, x_star, hyp_l, hyp_sigma_f, hyp_sigma_n);
  Eigen::MatrixXd k_star_star = sqExpKernel(x_star, x_star, hyp_l, hyp_sigma_f, hyp_sigma_n);

  // Algorithm 2.1 from Rasmussen & Williams
  // FIXME: Check that everything is correct and optmize (find cool Eigen functions)
  Eigen::LLT<Eigen::MatrixXd> l(k + std::pow(hyp_sigma_n, 2) * Eigen::MatrixXd::Identity(k.rows(), k.cols()));
  Eigen::MatrixXd alpha = l.matrixL().transpose().solve(l.matrixL().solve(y));

  double posterior_mean = (k_star.transpose() * alpha)(0, 0);

  Eigen::MatrixXd v = l.matrixL().solve(k_star);
  double posterior_variance = (k_star_star - (v.transpose() * v)).diagonal()(0, 0);

  return std::make_pair(posterior_mean, posterior_variance);
}
}  // namespace pig