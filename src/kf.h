#ifndef UNSCENTEDKF_KF_H
#define UNSCENTEDKF_KF_H

#include "measurement_package.h"
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

  void SetState(VectorXd &x_in);

  void SetProcessMatrix(MatrixXd &P_in);

  VectorXd GetState();

  MatrixXd GetProcessMatrix();

  /**
   * Updates the state by using standard Kalman Filter equations
   * @param z The measurement at k+1
   */
  void Update(const MeasurementPackage meas_package);

private:
  Tools tools;

  /**
   * Utility function for code reusing
   * @param z_diff
   */
  void EstimateState(const VectorXd &z_diff);
};


#endif //UNSCENTEDKF_KF_H
