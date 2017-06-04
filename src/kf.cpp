#include "kf.h"
#include <iostream>

using namespace std;
using Eigen::MatrixXd;
using Eigen::VectorXd;
using std::vector;

KF::KF() {
}

KF::~KF() {}

void KF::init(VectorXd &x_in, MatrixXd &P_in, MatrixXd &F_in,
              MatrixXd &H_in, MatrixXd &R_in, MatrixXd &Q_in) {
  x_= x_in;
  P_ = P_in;
  F_ = F_in;
  H_ = H_in;
  R_ = R_in;
  Q_ = Q_in;
}

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
