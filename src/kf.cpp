#include "kf.h"
#include <iostream>

using namespace std;
using Eigen::MatrixXd;
using Eigen::VectorXd;
using std::vector;

KF::KF() {
  std_laspx_ = 0.15;

  std_laspy_ = 0.15;

  R_ = MatrixXd::Zero(2, 2);
  R_(0, 0) = pow(std_laspx_, 2);
  R_(1, 1) = pow(std_laspy_, 2);

  H_ = MatrixXd::Zero(2, 5);

  H_(0, 0) = 1;
  H_(1, 1) = 1;

  H_k_ = H_;

  H_k_t = H_k_.transpose();

  I = MatrixXd::Identity(5, 5);
}

KF::~KF() {}

void KF::SetState(VectorXd &x_in) {
  x_= x_in;
}

void KF::SetProcessMatrix(MatrixXd &P_in) {
  P_ = P_in;
}

VectorXd KF::GetState() {
  return x_;
}

MatrixXd KF::GetProcessMatrix() {
  return P_;
}

void KF::Update(const MeasurementPackage meas_package) {
  /**
    * update the state by using Kalman Filter equations
  */
  VectorXd z = meas_package.raw_measurements_;
  VectorXd y = z - H_ * x_;
  EstimateState(y);
}

void KF::EstimateState(const VectorXd &z_diff) {
  MatrixXd S = H_k_ * P_ * H_k_t + R_;
  MatrixXd Si = S.inverse();
  MatrixXd PHt = P_ * H_k_t;

  MatrixXd K = PHt * Si;

  // New estimate
  x_ = x_ + (K * z_diff);
  P_ = (I - K * H_k_) * P_;
}
