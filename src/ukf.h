#ifndef UKF_H
#define UKF_H

#include "measurement_package.h"
#include "Eigen/Dense"
#include "FusionUKF.h"
#include "kf.h"
#include <vector>
#include <string>
#include <fstream>

using Eigen::MatrixXd;
using Eigen::VectorXd;

class UKF {
public:

  ///* initially set to false, set to true in first call of ProcessMeasurement
  bool is_initialized_;

  ///* state vector: [pos1 pos2 vel_abs yaw_angle yaw_rate] in SI units and rad
  VectorXd x_;

  ///* state covariance matrix
  MatrixXd P_;

  ///* time when the state is true, in us
  long long time_us_;

  ///* State dimension
  int n_x_;

  /**
   * Constructor
   */
  UKF();

  /**
   * Destructor
   */
  virtual ~UKF();

  /**
   * ProcessMeasurement
   * @param meas_package The latest measurement data of either radar or laser
   */
  void ProcessMeasurement(MeasurementPackage meas_package);

private:
  FusionUKF fusionUKF;

  KF KF;

  Tools tools;

  MeasurementPackage::SensorType _GetSensorType(
          const MeasurementPackage &measurement_pack
  );

  /**
   * Prediction Predicts sigma points, the state, and the state covariance
   * matrix for Radar
   * @param delta_t Time between k and k+1 in s
   */
  void _Prediction(double_t delta_t);

  /**
   * Updates the state and the state covariance matrix using a laser measurement
   * @param meas_package The measurement at k+1
   */
  void _UpdateLidar(MeasurementPackage meas_package);
  /**
   * Updates the state and the state covariance matrix using a radar measurement
   * @param meas_package The measurement at k+1
   */
  void _UpdateRadar(MeasurementPackage meas_package);
};

#endif /* UKF_H */
