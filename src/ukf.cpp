#include "ukf.h"
#include <iostream>

using namespace std;
using Eigen::MatrixXd;
using Eigen::VectorXd;
using std::vector;

/**
 * Initializes Unscented Kalman filter
 */
UKF::UKF() {
  /**
  Complete the initialization. See ukf.h for other member properties.
  */
  n_x_ = 5;

  is_initialized_ = false;

  x_ = VectorXd::Zero(n_x_);

  P_ = MatrixXd::Identity(n_x_, n_x_);
}

UKF::~UKF() {}

/**
 * @param {MeasurementPackage} meas_package The latest measurement data of
 * either radar or laser.
 */
void UKF::ProcessMeasurement(MeasurementPackage meas_package) {
  /**
  Complete this function! Make sure you switch between lidar and radar
  measurements.
  */
  MeasurementPackage::SensorType sensorType = _GetSensorType(meas_package);

  if(!is_initialized_) {
    switch (sensorType) {
      case MeasurementPackage::RADAR:
        /**
         * Test only
         */
        x_(0) = tools.Polar2Cart(meas_package)(0);
        x_(1) = tools.Polar2Cart(meas_package)(1);
        fusionUKF.SetState(x_);
        fusionUKF.SetProcessMatrix(P_);
        break;
      case MeasurementPackage::LASER:
        x_(0) = meas_package.raw_measurements_(0);
        x_(1) = meas_package.raw_measurements_(1);
        KF.SetState(x_);
        KF.SetProcessMatrix(P_);
        break;
      default:
        break;
    }
    time_us_ = meas_package.timestamp_;
    is_initialized_ = true;
    return;
  }

  // in seconds
  double_t delta_t = tools.GetTimeDiff(meas_package.timestamp_, time_us_);
  time_us_ = meas_package.timestamp_;

  _Prediction(delta_t);

  switch (sensorType) {
    case MeasurementPackage::RADAR:
      _UpdateRadar(meas_package);
      break;
    case MeasurementPackage::LASER:
      _UpdateLidar(meas_package);
      break;
    default:
      break;
  }

  std::cout << "x_out:" << x_ << std::endl;
  std::cout << "P_out:" << P_ << std::endl;
}

/**
 * Predicts sigma points, the state, and the state covariance matrix.
 * @param {double_t} delta_t the change in time (in seconds) between the last
 * measurement and this one.
 */
void UKF::_Prediction(double_t delta_t) {
  /**
  Complete this function! Estimate the object's location. Modify the state
  vector, x_. Predict sigma points, the state, and the state covariance matrix.
  */
  double_t threshold = 1e-3;
  if (delta_t <= threshold) {
    return;
  }
  fusionUKF.SetState(x_);
  fusionUKF.SetProcessMatrix(P_);
  fusionUKF.Predict(delta_t);
  x_ = fusionUKF.GetState();
  P_ = fusionUKF.GetProcessMatrix();
  // Update Lidar prediction state
  KF.SetState(x_);
  KF.SetProcessMatrix(P_);
}

/**
 * Updates the state and the state covariance matrix using a laser measurement.
 * @param {MeasurementPackage} meas_package
 */
void UKF::_UpdateLidar(MeasurementPackage meas_package) {
  /**
  Complete this function! Use lidar data to update the belief about the object's
  position. Modify the state vector, x_, and covariance, P_.

  You'll also need to calculate the lidar NIS.
  */
  KF.Update(meas_package);
  x_ = KF.GetState();
  P_ = KF.GetProcessMatrix();
}

/**
 * Updates the state and the state covariance matrix using a radar measurement.
 * @param {MeasurementPackage} meas_package
 */
void UKF::_UpdateRadar(MeasurementPackage meas_package) {
  /**
  Complete this function! Use radar data to update the belief about the object's
  position. Modify the state vector, x_, and covariance, P_.

  You'll also need to calculate the radar NIS.
  */
  fusionUKF.Update(meas_package);
  x_ = fusionUKF.GetState();
  P_ = fusionUKF.GetProcessMatrix();
}

MeasurementPackage::SensorType UKF::_GetSensorType(
        const MeasurementPackage &measurement_pack) {
  return measurement_pack.sensor_type_;
}
