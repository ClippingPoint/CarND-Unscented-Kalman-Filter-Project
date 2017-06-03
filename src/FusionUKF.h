#ifndef UNSCENTEDKF_FUSIONUKF_H
#define UNSCENTEDKF_FUSIONUKF_H

#include "measurement_package.h"
#include "tools.h"
#include "Eigen/Dense"
#include <vector>
#include <string>
#include <fstream>
#include <cmath>

using Eigen::MatrixXd;
using Eigen::VectorXd;

class FusionUKF {
public:

  VectorXd x_;

  MatrixXd P_;

  ///* predicted sigma points matrix
  MatrixXd Xsig_pred_;

  // Cache intermediate results
  MatrixXd x_pred_;

  MatrixXd P_pred_;

  MatrixXd X_diff_;

  MatrixXd Z_diff_;

  ///* Process noise standard deviation longitudinal acceleration in m/s^2
  double_t std_a_;

  ///* Process noise standard deviation yaw acceleration in rad/s^2
  double_t std_yawdd_;

  ///* Radar measurement noise standard deviation radius in m
  double_t std_radr_;

  ///* Radar measurement noise standard deviation angle in rad
  double_t std_radphi_;

  ///* Radar measurement noise standard deviation radius change in m/s
  double_t std_radrd_;

  ///* Weights of sigma points
  VectorXd weights_;

  ///* State dimension
  int n_x_;

  ///* Augmented state dimension
  int n_aug_;

  ///* Observation dimension
  int n_z_;

  ///* Sigma point spreading parameter
  double_t lambda_;

  /**
   * Constructor
   */
  FusionUKF();

  /**
   * Destructor
   */
  virtual ~FusionUKF();

  void Init(MeasurementPackage meas_package);

  void SetState(const VectorXd &x_set);
  void SetProcessMatrix(const MatrixXd &P_set);
  VectorXd GetState();
  MatrixXd GetProcessMatrix();

  /**
   * @param delta_t
   */
  void PredictRadar(double_t delta_t);

  /**
   * @param meas_package
   */
  void UpdateRadar(const MeasurementPackage meas_package);

private:
  Tools tools;

  VectorXd _GenerateWeights(int dim);

  MatrixXd _GenerateSigmaPoints();

  /**
   *
   * @param Xsig_aug
   */
  void _MotionPrediction(MatrixXd &Xsig_aug, double_t delta_t);

  MatrixXd _PredictMeanAndCovariance(VectorXd* x_out, MatrixXd* P_out,
                                 int norm_dim, MatrixXd &SIG);

  void _PropagateNoise(MatrixXd *S);

  MatrixXd _GetCrossCovariance(MatrixXd &X_diff, MatrixXd &Z_diff);

  void _AugmentStateAndProcess(VectorXd *x_out, MatrixXd* P_out);
};


#endif //UNSCENTEDKF_FUSIONUKF_H
