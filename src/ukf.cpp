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
  // if this is false, laser measurements will be ignored (except during init)
  use_laser_ = true;

  // if this is false, radar measurements will be ignored (except during init)
  use_radar_ = true;

  /**
  TODO:

  Complete the initialization. See ukf.h for other member properties.

  Hint: one or more values initialized above might be wildly off...
  */
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
  MeasurementPackage::SensorType sensorType = GetSensorType(meas_package);
  /**
   * TODO: Refactor initialization assignment
   * Polar to Cart position initialization
   */
  if(!is_initialized_) {
    switch (sensorType) {
      case MeasurementPackage::RADAR:
        fusionUKF.Init(meas_package);
        x_ = fusionUKF.x_;
        P_ = fusionUKF.P_;
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
  double_t threshold = 1e-3;

  switch (sensorType) {
    case MeasurementPackage::RADAR:
      break;
    case MeasurementPackage::LASER:
      return;
//      break;
    default:
      break;
  }
  if (delta_t >= threshold) {
    Prediction(delta_t);
  }

  UpdateRadar(meas_package);
}

/**
 * Predicts sigma points, the state, and the state covariance matrix.
 * @param {double} delta_t the change in time (in seconds) between the last
 * measurement and this one.
 */
void UKF::Prediction(double delta_t) {
  /**
  TODO:

  Complete this function! Estimate the object's location. Modify the state
  vector, x_. Predict sigma points, the state, and the state covariance matrix.
  */
  fusionUKF._PredictRadar(delta_t, x_, P_);
}

/**
 * Updates the state and the state covariance matrix using a laser measurement.
 * @param {MeasurementPackage} meas_package
 */
void UKF::UpdateLidar(MeasurementPackage meas_package) {
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
void UKF::UpdateRadar(MeasurementPackage meas_package) {
  /**
  TODO:

  Complete this function! Use radar data to update the belief about the object's
  position. Modify the state vector, x_, and covariance, P_.

  You'll also need to calculate the radar NIS.
  */
  fusionUKF._UpdateRadar(meas_package);
  x_ = fusionUKF.GetState();
  P_ = fusionUKF.GetProcessMatrix();

  std::cout << "x_out:" << x_ << std::endl;
  std::cout << "P_out:" << P_ << std::endl;
}

MeasurementPackage::SensorType UKF::GetSensorType(
        const MeasurementPackage &measurement_pack) {
  return measurement_pack.sensor_type_;
}
