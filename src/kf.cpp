#include "kf.h"
#include <iostream>
#include "kf.h"
#include "tools.h"
#include "Eigen/Dense"

using namespace std;
using Eigen::MatrixXd;
using Eigen::VectorXd;
using std::vector;

KF::KF() {}

KF::~KF() {}

void KF::Predict() {
  /**
    * predict the state
  */
  x_ = F_ * x_;
  P_ = F_ * P_ * Ft + Q_;
}

void KF::Update(const VectorXd &z) {
  /**
    * update the state by using Kalman Filter equations
  */
  VectorXd y = z - H_ * x_;
  EstimateState(y);
}

void KF::UpdateEKF(const VectorXd &z) {
  /**
    * update the state by using Extended Kalman Filter equations
  */
  VectorXd y = z - H_;

  y = tools.NormalizeAngleVec(y, 1);
  EstimateState(y);
}

void KF::EstimateState(const Eigen::VectorXd &z_diff) {
//  MatrixXd Ht = H_k_.transpose();
  MatrixXd S = H_k_ * P_ * H_k_t + R_;
  MatrixXd Si = S.inverse();
  MatrixXd PHt = P_ * H_k_t;

  MatrixXd K = PHt * Si;

  // New estimate
  x_ = x_ + (K * z_diff);
  P_ = (I - K * H_k_) * P_;
}
