#include "ukf.h"
#include "Eigen/Dense"
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

  Hint: one or more values initialized above might be wildly off...
  */
  n_x_ = 5;

  is_initialized_ = false;

  x_ = VectorXd::Zero(n_x_);

  // initial covariance matrix
  P_ = MatrixXd::Zero(n_x_, n_x_);
}

UKF::~UKF() {}

/**
 * @param {MeasurementPackage} meas_package The latest measurement data of
 * either radar or laser.
 */
void UKF::ProcessMeasurement(MeasurementPackage meas_package) {
  /**
  TODO:

  Complete this function! Make sure you switch between lidar and radar
  measurements.
  */
  MeasurementPackage::SensorType sensorType = _GetSensorType(meas_package);

  /**
   * TODO: Refactor initialization assignment
   * Polar to Cart position initialization
   */
  if(!is_initialized_) {
    switch (sensorType) {
      case MeasurementPackage::RADAR:
        fusionUKF.Init(meas_package);
        x_ = fusionUKF.GetState();
        P_ = fusionUKF.GetProcessMatrix();
        break;
      case MeasurementPackage::LASER:
        return;
//        break;
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

  switch (sensorType) {
    case MeasurementPackage::RADAR:
      _UseRadar(meas_package, delta_t);
      break;
    case MeasurementPackage::LASER:
      return;
      _UseLidar(meas_package, delta_t);
      break;
    default:
      break;
  }
  std::cout << "x_out:" << x_ << std::endl;
  std::cout << "P_out:" << P_ << std::endl;
}

// TODO: Refactor below two functions?
/**
 * @param meas_package
 * @param delta_t
 */
void UKF::_UseLidar(MeasurementPackage meas_package, double_t delta_t) {
  _PredictLidar(delta_t);
  _UpdateLidar(meas_package);
}

/**
 * @param meas_package
 * @param delta_t
 */
void UKF::_UseRadar(MeasurementPackage meas_package, double_t delta_t){
  _PredictRadar(delta_t);
  _UpdateRadar(meas_package);
}

/**
 * Prediction step for Lidar
 * @param delta_t
 */
void UKF::_PredictLidar(double_t delta_t) {
  double_t threshold = 1e-3;
  if (delta_t <= threshold) {
    return;
  }
}

/**
 * Predicts sigma points, the state, and the state covariance matrix.
 * @param {double_t} delta_t the change in time (in seconds) between the last
 * measurement and this one.
 */
void UKF::_PredictRadar(double_t delta_t) {
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
  fusionUKF.PredictRadar(delta_t);
}

/**
 * Updates the state and the state covariance matrix using a laser measurement.
 * @param {MeasurementPackage} meas_package
 */
void UKF::_UpdateLidar(MeasurementPackage meas_package) {
  /**
  TODO:

  Complete this function! Use lidar data to update the belief about the object's
  position. Modify the state vector, x_, and covariance, P_.

  You'll also need to calculate the lidar NIS.
  */
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
  fusionUKF.UpdateRadar(meas_package);
  x_ = fusionUKF.GetState();
  P_ = fusionUKF.GetProcessMatrix();
}

MeasurementPackage::SensorType UKF::_GetSensorType(
        const MeasurementPackage &measurement_pack) {
  return measurement_pack.sensor_type_;
}
