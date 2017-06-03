#include <iostream>
#include "tools.h"

using Eigen::VectorXd;
using Eigen::MatrixXd;
using std::vector;

Tools::Tools() {}

Tools::~Tools() {}

VectorXd Tools::CalculateRMSE(const vector<VectorXd> &estimations,
                              const vector<VectorXd> &ground_truth) {
  /**
  TODO:
    * Calculate the RMSE here.
  */
  VectorXd rmse(4);
  rmse << 0, 0, 0, 0;

  // Check the validity of the following inputs:
  // * the estimation vector size should not be zero
  // * the estimation vector size should equal ground truth vector size

  if (estimations.size() != ground_truth.size()
      || estimations.size() == 0) {
    cout << "Invalid estimation or ground_truth data" << endl;
    return rmse;
  }

  // accumulate squared residuals
  for (unsigned int i=0; i < estimations.size(); i+=1) {
    VectorXd residual = estimations[i] - ground_truth[i];

    // Coefficient-wise multiplication
    residual = pow(residual.array(), 2);
    rmse += residual;
  }

  // Calculate the mean
  rmse = rmse/estimations.size();

  // Calculate the squared root
  rmse = rmse.array().sqrt();

  return rmse;
}

MatrixXd Tools::NormalizeAngle(MatrixXd &Z_diff, int dim) {
  double_t TWO_PI = 2. * M_PI;
  int width = Z_diff.cols();
  for (int i = 0; i < width; i+=1) {
    while(Z_diff.col(i)(dim) > M_PI) { Z_diff.col(i)(dim) -= TWO_PI; }
    while(Z_diff.col(i)(dim) < -M_PI) { Z_diff.col(i)(dim) += TWO_PI; }
  }
  return Z_diff;
}

VectorXd Tools::NormalizeAngleVec(VectorXd &z_diff, int dim) {
  double_t TWO_PI = 2. * M_PI;
  while(z_diff(dim) > M_PI) { z_diff(dim) -= TWO_PI; }
  while(z_diff(dim) < -M_PI) { z_diff(dim) += TWO_PI; }
  return z_diff;
}


/**
 *
 * @param Xsig Reuse Sigma Point generated for state estimation
 * @return
 */
MatrixXd Tools::Cart2Polar(const MatrixXd &Xsig) {
  int width = Xsig.cols();
  MatrixXd H_x = MatrixXd::Zero(3, width);

  VectorXd p_x = Xsig.row(0);
  VectorXd p_y = Xsig.row(1);
  VectorXd v = Xsig.row(2);
  VectorXd yaw = Xsig.row(3);

  VectorXd v1 = cos(yaw.array()) * v.array();
  VectorXd v2 = sin(yaw.array()) * v.array();
  VectorXd r = sqrt(p_x.array().pow(2) + p_y.array().pow(2));
  // How to do element wise atan2 in eigen?
//  VectorXd phi = atan2(p_y.array(), p_x.array());
  VectorXd phi = VectorXd::Zero(width);
  VectorXd r_dot = VectorXd::Zero(width);

  double_t threshold = 1e-3;
  for (int i = 0; i < width; i+=1) {
    if (r(i) < threshold) {
      r_dot(i) = 0;
    } else {
      r_dot(i) = (p_x(i) * v1(i) + p_y(i) * v2(i))/r(i);
    }
    phi(i) = atan2(p_y(i), p_x(i));
  }
  H_x.row(0) = r;
  H_x.row(1) = phi;
  H_x.row(2) = r_dot;
  return H_x;
}

/**
 * Time diff in seconds
 * @param curr_time
 * @param prev_time
 * @return
 */
double_t Tools::GetTimeDiff(long long curr_time, long long prev_time) {
  return (double_t)(curr_time - prev_time)/1000000.0;
}
