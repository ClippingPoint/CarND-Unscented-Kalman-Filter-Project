#include "FusionUKF.h"
#include "Eigen/Dense"
#include <iostream>

using namespace std;
using Eigen::MatrixXd;
using Eigen::VectorXd;


FusionUKF::FusionUKF() {
  /**
   * Use augmented state
   */
  n_aug_ = 7;

  n_x_ = 5;

  lambda_ = 3;

  x_ = VectorXd::Zero(n_x_);

  P_ = MatrixXd::Identity(n_x_, n_x_);

  Xsig_pred_ = MatrixXd::Zero(n_x_, 2 * n_aug_ + 1);

  // Process noise standard deviation longitudinal acceleration in m/s^2
  //std_a_ = 0.2;
  std_a_ = 0.6;

  // Process noise standard deviation yaw acceleration in rad/s^2
  //std_yawdd_ = 0.2;
  std_yawdd_ = 0.9;

  // Laser measurement noise standard deviation position1 in m
  std_laspx_ = 0.15;

  // Laser measurement noise standard deviation position2 in m
  std_laspy_ = 0.15;

  // Radar measurement noise standard deviation radius in m
  std_radr_ = 0.3;

  // Radar measurement noise standard deviation angle in rad
  std_radphi_ = 0.03;
//  std_radphi_ = 0.0175;

  // Radar measurement noise standard deviation radius change in m/s
  std_radrd_ = 0.3;
//  std_radrd_ = 0.1;

  // Radar
  n_z_ = 3;

  weights_ = _GenerateWeights(n_aug_);
}

FusionUKF::~FusionUKF() {}


void FusionUKF::Init(MeasurementPackage meas_package) {
  /**
   * Test only
   */
  /*x_(0) = 5.7441;
  x_(1) = 1.3800;
  x_(2) = 2.2049;
  x_(3) = 0.5015;
  x_(4) = 0.3528;*/
  VectorXd x_set = VectorXd::Zero(n_x_);
  x_set(0) = tools.Polar2Cart(meas_package)(0);
  x_set(1) = tools.Polar2Cart(meas_package)(1);
  x_set(2) = 2.2049;
  x_set(3) = 0.5015;
  x_set(4) = 0.3528;
  SetState(x_set);
  MatrixXd P_set = MatrixXd::Identity(n_x_, n_x_);
/*  P_set << 0.0043, -0.0013, 0.0030, -0.0022, -0.0020,
        -0.0013, 0.0077, 0.0011, 0.0071, 0.0060,
        0.0030, 0.0011, 0.0054, 0.0007, 0.0008,
        -0.0022, 0.0071, 0.0007, 0.0098, 0.0010,
        -0.0020, 0.0060, 0.0008, 0.0100, 0.0123;*/
  SetProcessMatrix(P_set);
}


VectorXd FusionUKF::_GenerateWeights(int dim) {
  VectorXd weights = VectorXd::Zero(2 * dim + 1);
  weights.fill(0.5/lambda_);
  weights(0) = (lambda_ - n_aug_)/lambda_;
  return weights;
}

void FusionUKF::SetState(const VectorXd &x_set) {
  x_ = x_set;
}

void FusionUKF::SetProcessMatrix(const MatrixXd &P_set) {
  P_ = P_set;
}

VectorXd FusionUKF::GetState() {
  return x_;
}

MatrixXd FusionUKF::GetProcessMatrix() {
  return P_;
}

void FusionUKF::_AugmentStateAndProcess(VectorXd *x_out, MatrixXd *P_out) {
  VectorXd x_aug = VectorXd::Zero(n_aug_);
  x_aug(0) = x_(0);
  x_aug(1) = x_(1);
  x_aug(2) = x_(2);
  x_aug(3) = x_(3);
  x_aug(4) = x_(4);
  x_aug(5) = 0;
  x_aug(6) = 0;

  MatrixXd P_aug = MatrixXd::Zero(n_aug_, n_aug_);
  P_aug.topLeftCorner(n_x_, n_x_) = P_;
  P_aug(5, 5) = pow(std_a_, 2);
  P_aug(6, 6) = pow(std_yawdd_, 2);
  *x_out = x_aug;
  *P_out = P_aug;
}

/**
 *
 * @return
 */
MatrixXd FusionUKF::_GenerateSigmaPoints() {

  VectorXd x_aug = VectorXd::Zero(n_aug_);
  MatrixXd P_aug = MatrixXd::Zero(n_aug_, n_aug_);
  _AugmentStateAndProcess(&x_aug, &P_aug);
  MatrixXd L = P_aug.llt().matrixL();

  // create augmented sigma points
  MatrixXd Xsig_aug = MatrixXd::Zero(n_aug_, 2 * n_aug_ + 1);

//  MatrixXd row_vector = MatrixXd::Ones(1, n_aug_);
//  Column duplication
  VectorXd row_vector = VectorXd::Ones(n_aug_);

  // Outer product
  //x_mat.noalias() = x_ * row_vector.transpose();
  MatrixXd x_mat = x_aug * row_vector.transpose();

  MatrixXd left_block = MatrixXd::Zero(n_aug_, n_aug_);
  left_block = x_mat + sqrt(lambda_) * L;

  MatrixXd right_block = MatrixXd::Zero(n_aug_, n_aug_);
  right_block = x_mat - sqrt(lambda_) * L;

  Xsig_aug.col(0) = x_aug;
  Xsig_aug.block(0, 1, n_aug_, n_aug_)  = left_block;
  Xsig_aug.block(0, n_aug_ + 1, n_aug_, n_aug_) = right_block;
  return Xsig_aug;
}

/**
 * Non linear mapping
 * @param Xsig_aug
 */
void FusionUKF::_MotionPrediction(MatrixXd &Xsig_aug, double_t delta_t){
  /**
   * Test only
   */
//  delta_t = 0.1;
  VectorXd p_x = Xsig_aug.row(0);
  VectorXd p_y = Xsig_aug.row(1);
  VectorXd v = Xsig_aug.row(2);
  VectorXd yaw = Xsig_aug.row(3);
  VectorXd yawd = Xsig_aug.row(4);
  VectorXd nu_a = Xsig_aug.row(5);
  VectorXd nu_yawdd = Xsig_aug.row(6);

  double_t threshold = 1e-3;
  int vec_len = 2 * n_aug_ + 1;
  VectorXd px_p = VectorXd::Zero(vec_len);
  VectorXd py_p = VectorXd::Zero(vec_len);

  VectorXd v_p = VectorXd::Zero(vec_len);
  VectorXd yaw_p = VectorXd::Zero(vec_len);
  VectorXd yawd_p = VectorXd::Zero(vec_len);

  for (int i = 0; i < vec_len; i+=1) {
    //avoid division by zero
    if (fabs(yawd(i)) > threshold) {
      px_p(i) = p_x(i) + v(i)/yawd(i) * ( sin (yaw(i) + yawd(i)*delta_t) - sin(yaw(i)));
      py_p(i) = p_y(i) + v(i)/yawd(i) * ( cos(yaw(i)) - cos(yaw(i)+yawd(i)*delta_t) );
    }
    else {
      px_p(i) = p_x(i) + v(i)*delta_t*cos(yaw(i));
      py_p(i) = p_y(i) + v(i)*delta_t*sin(yaw(i));
    }

    v_p(i) = v(i);
    yaw_p(i) = yaw(i) + yawd(i)*delta_t;
    yawd_p(i) = yawd(i);

    /**
     * TODO: How to do this element wise
     */
    //add noise
    px_p(i) = px_p(i) + 0.5*nu_a(i)*delta_t*delta_t * cos(yaw(i));
    py_p(i) = py_p(i) + 0.5*nu_a(i)*delta_t*delta_t * sin(yaw(i));
    v_p(i) = v_p(i) + nu_a(i)*delta_t;
  }

  yaw_p = yaw_p + 0.5*nu_yawdd*delta_t*delta_t;
  yawd_p = yawd_p + nu_yawdd*delta_t;
  // Element wise operations
  Xsig_pred_.row(0) = px_p;
  Xsig_pred_.row(1) = py_p;
  Xsig_pred_.row(2) = v_p;
  Xsig_pred_.row(3) = yaw_p;
  Xsig_pred_.row(4) = yawd_p;
}
/**
 *
 * (Xsig_pred_ * w).tranpose()
 * @param x_out
 * @param P_out
 * @param norm_dim state/measurement vector dim needs to be normalized
 * @param SIG
 * State: Xsig_pred_
 * Measurement: Zsig
 * @return X - x_ or Z - z_
 */
MatrixXd FusionUKF::_PredictMeanAndCovariance(VectorXd *x_out, MatrixXd *P_out,
                                          int norm_dim, MatrixXd &SIG) {
  VectorXd x = SIG * weights_;
//  auto W?
  MatrixXd W = weights_.asDiagonal();
  // Column Duplication
  VectorXd row_vector = VectorXd::Ones(2 * n_aug_ + 1);

  MatrixXd x_mat = x * row_vector.transpose();
  MatrixXd X_diff = SIG - x_mat;

  // Normalization
  // For State  Xsig_pred 3: yaw
  // For Measurement Zsig 1: phi
  tools.NormalizeAngle(X_diff, norm_dim);
  MatrixXd P = X_diff * W * X_diff.transpose();

  *x_out = x;
  *P_out = P;
  return X_diff;
}

/**
 * Propagate additive measurement noise
 * @param S
 */
void FusionUKF::_PropagateNoise(MatrixXd *S) {
  MatrixXd R = MatrixXd::Zero(n_z_, n_z_);
  R(0, 0) = pow(std_radr_, 2);
  R(1, 1) = pow(std_radphi_, 2);
  R(2, 2) = pow(std_radrd_, 2);
  *S = *S + R;
}

/**
 * X_diff * W * Z_diff.T
 * @param X_diff
 * @param Z_diff
 * @return
 */
MatrixXd FusionUKF::_GetCrossCovariance(MatrixXd &X_diff, MatrixXd &Z_diff) {
  return X_diff * weights_.asDiagonal() * Z_diff.transpose();
}

void FusionUKF::_PredictRadar(double_t delta_t) {
  MatrixXd Xsig_aug = _GenerateSigmaPoints();
  _MotionPrediction(Xsig_aug, delta_t);

  VectorXd x_pred = VectorXd::Zero(n_aug_);
  MatrixXd P_pred = MatrixXd::Zero(n_aug_, n_aug_);
  X_diff_ = _PredictMeanAndCovariance(&x_pred, &P_pred,
                                     3, Xsig_pred_);

  x_pred_ = x_pred;
  P_pred_ = P_pred;

}

void FusionUKF::_UpdateRadar(MeasurementPackage meas_package) {
  VectorXd z_pred = VectorXd::Zero(n_z_);
  MatrixXd S = MatrixXd::Zero(n_z_, n_z_);
  MatrixXd Zsig = tools.Cart2Polar(Xsig_pred_);
  Z_diff_ = _PredictMeanAndCovariance(&z_pred, &S, 1, Zsig);
  _PropagateNoise(&S);

  // Kalman Update Process
  MatrixXd Tc = _GetCrossCovariance(X_diff_, Z_diff_);
  MatrixXd K = Tc * S.inverse();
  /**
   * Test only
   */
/*  VectorXd z = VectorXd::Zero(n_z_);
  z << 5.9214, 0.2187, 2.0062;*/
  VectorXd z_diff = meas_package.raw_measurements_ - z_pred;
  // angle normalization
  z_diff = tools.NormalizeAngleVec(z_diff, 1);

  VectorXd x = x_pred_ + K * z_diff;
  MatrixXd P = P_pred_ - K * S * K.transpose();

  SetState(x);
  SetProcessMatrix(P);
}

