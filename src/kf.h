//
// Created by ian zhang on 6/3/17.
//

#ifndef UNSCENTEDKF_KF_H
#define UNSCENTEDKF_KF_H

#include "Eigen/Dense"
#include "tools.h"

using Eigen::MatrixXd;
using Eigen::VectorXd;

class KF {
public:

  // state vector
  VectorXd x_;

  // state covariance matrix
  MatrixXd P_;

  // state transition matrix
  MatrixXd F_;

  // state transition matrix transpose for reusing calc result
  MatrixXd Ft;

  // process covariance matrix
  MatrixXd Q_;

  // measurement matrix
  MatrixXd H_;

  // measurement covariance matrix
  MatrixXd R_;

  // measurement matrix for kalman gain
  MatrixXd H_k_;

  // Cache variable
  MatrixXd H_k_t;

  // Identity
  MatrixXd I;

  ///* Laser measurement noise standard deviation position1 in m
  double_t std_laspx_;

  ///* Laser measurement noise standard deviation position2 in m
  double_t std_laspy_;

  /**
   * Constructor
   */
  KF();

  /**
   * Destructor
   */
  virtual ~KF();

  void init(VectorXd &x_in, MatrixXd &P_in);

  /**
   * Updates the state by using standard Kalman Filter equations
   * @param z The measurement at k+1
   */
  void Update(const Eigen::VectorXd &z);

private:
  Tools tools;

  /**
   * Utility function for code reusing
   * @param z_diff
   */
  void EstimateState(const Eigen::VectorXd &z_diff);
};


#endif //UNSCENTEDKF_KF_H
