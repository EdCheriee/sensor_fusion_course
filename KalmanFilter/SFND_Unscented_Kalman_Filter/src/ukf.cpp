#include "ukf.h"
#include "Eigen/Dense"
#include <iostream>

using Eigen::MatrixXd;
using Eigen::VectorXd;

/**
 * Initializes Unscented Kalman filter
 */
UKF::UKF() 
{
  // if this is false, laser measurements will be ignored (except during init)
  use_laser_ = true;

  // if this is false, radar measurements will be ignored (except during init)
  use_radar_ = true;

  // initial state vector
  x_ = VectorXd(5);
  x_.fill(0.0);

  // initial covariance matrix
  P_ = MatrixXd(5, 5);
  P_.fill(0.0);

  // Process noise standard deviation longitudinal acceleration in m/s^2
  std_a_ = 2;

  // Process noise standard deviation yaw acceleration in rad/s^2
  std_yawdd_ = 1;
  
  /**
   * DO NOT MODIFY measurement noise values below.
   * These are provided by the sensor manufacturer.
   */

  // Laser measurement noise standard deviation position1 in m
  std_laspx_ = 0.15;

  // Laser measurement noise standard deviation position2 in m
  std_laspy_ = 0.15;

  // Radar measurement noise standard deviation radius in m
  std_radr_ = 0.3;

  // Radar measurement noise standard deviation angle in rad
  std_radphi_ = 0.03;

  // Radar measurement noise standard deviation radius change in m/s
  std_radrd_ = 0.3;
  
  /**
   * End DO NOT MODIFY section for measurement noise values 
   */
  
  /**
   * TODO: Complete the initialization. See ukf.h for other member properties.
   * Hint: one or more values initialized above might be wildly off...
   */
  n_x_ = 5;
  n_aug_ = 7;
  lambda_ = 3 - n_aug_;
  time_us_ = 0.0l;

  Xsig_pred_ = MatrixXd(n_x_, 2 * n_aug_ + 1);

  weights_ = VectorXd(2 * n_aug_ + 1);
  double weight_0 = lambda_/(lambda_ + n_aug_);
  double weight = 0.5/(lambda_ + n_aug_);
  weights_(0) = weight_0;

  for (int i=1; i < 2 * n_aug_ + 1; ++i) 
  {  
    weights_(i) = weight;
  }

  is_initialized_ = false;

  NIS_lidar_ = 0.0;
  NIS_radar_ = 0.0;
}

UKF::~UKF() {}

void UKF::ProcessMeasurement(MeasurementPackage meas_package) 
{
  /**
   * TODO: Complete this function! Make sure you switch between lidar and radar
   * measurements.
   */

  // Initialise the state vector and the covariance matrix. Covarian matrix could
  // be initialised to stds^2 instead.
  if(!is_initialized_)
  {
    if(meas_package.sensor_type_ == MeasurementPackage::RADAR)
    {
      auto rho = meas_package.raw_measurements_[0];
      auto phi = meas_package.raw_measurements_[1];
      auto rho_dot = meas_package.raw_measurements_[2];

      auto px = rho * sin(phi);
      auto py = rho * cos(phi);

      x_ << px,
            py,
            rho_dot,
            phi,
            0;
      P_.setIdentity();
    }
    else if(meas_package.sensor_type_ == MeasurementPackage::LASER)
    {
      auto x = meas_package.raw_measurements_[0];
      auto y = meas_package.raw_measurements_[1];

      x_ << x,
            y,
            0,
            0,
            0;

      P_.setIdentity();
    }
    else
    {
      std::cerr << "Incorrect sensor type value.. EXITING" << std::endl;
      exit(-1);
    }
    is_initialized_ = true;
    // timestamp is already in microseconds
    time_us_ = meas_package.timestamp_;
  }

  //Change delta_t to seconds from microseconds
  auto delta_t = (meas_package.timestamp_ - time_us_) / 1e6;
  time_us_ = meas_package.timestamp_;

  Prediction(delta_t);

  if(meas_package.sensor_type_ == MeasurementPackage::RADAR)
    UpdateRadar(meas_package);
  else if (meas_package.sensor_type_ == MeasurementPackage::LASER)
    UpdateLidar(meas_package);
  else
  {
    std::cerr << "Incorrect sensor type value.. EXITING" << std::endl;
    exit(-1);
  }
}

void UKF::Prediction(double delta_t) 
{
  /**
   * TODO: Complete this function! Estimate the object's location. 
   * Modify the state vector, x_. Predict sigma points, the state, 
   * and the state covariance matrix.
   */

  // Create sigma points
  MatrixXd Xsig_aug = MatrixXd(n_aug_, 2 * n_aug_ + 1);
  GenerateAugmentedSigmaPoints(&Xsig_aug);

  // Predict sigma points
  PredictSigmaPoints(Xsig_aug, delta_t);

  // Predict new state mean and covariance matrix
  x_.fill(0.0);
  for (int i = 0; i < Xsig_pred_.cols(); i++) 
  {
    x_ = x_ + weights_(i) * Xsig_pred_.col(i);
  }

  P_.fill(0.0);
  for(auto j = 0; j < Xsig_pred_.cols(); j++)
  {
    VectorXd x_diff = Xsig_pred_.col(j) - x_;
    // angle normalization
    while (x_diff(3) > M_PI) 
      x_diff(3) -= 2.0 * M_PI;
    while (x_diff(3) < -M_PI) 
      x_diff(3) += 2.0 * M_PI;

    P_ += weights_(j) * x_diff * x_diff.transpose();
  }
}

void UKF::GenerateAugmentedSigmaPoints(MatrixXd* Xsig_out)
{
  // create augmented mean vector
  VectorXd x_aug = VectorXd(n_aug_);

  // create augmented state covariance
  MatrixXd P_aug = MatrixXd(n_aug_, n_aug_);

  // create sigma point matrix
  MatrixXd Xsig_aug = MatrixXd(n_aug_, 2 * n_aug_ + 1);

  // create augmented mean state
  x_aug.head(n_x_) = x_;
  x_aug(5) = 0;
  x_aug(6) = 0;

  // create augmented covariance matrix
  P_aug.fill(0.0);
  auto std_a_2 = std_a_ * std_a_;
  auto std_yawdd_2 = std_yawdd_ * std_yawdd_;
  MatrixXd Q = MatrixXd(2, 2);
  Q << std_a_2, 0,
        0, std_yawdd_2;
  P_aug.topLeftCorner(n_x_, n_x_) = P_;
  P_aug.bottomRightCorner(2, 2) = Q;
  // create square root matrix
  MatrixXd A_aug = P_aug.llt().matrixL();
  auto sqrtMatrix = sqrt((lambda_ + n_aug_)) * A_aug;

  // create augmented sigma points
  Xsig_aug.col(0) = x_aug;
  auto positiveSigmaPoints = sqrtMatrix.colwise() + x_aug;
  auto negativeSigmaPoints = (-1 * sqrtMatrix).colwise() + x_aug;

  for(int i = 0; i < positiveSigmaPoints.cols(); i++)
  {
    Xsig_aug.col(i + 1) = positiveSigmaPoints.col(i);
    Xsig_aug.col(i + 1 + n_aug_) = negativeSigmaPoints.col(i);
  }

  // Pass the augmented values out
  *Xsig_out = Xsig_aug;
}

void UKF::PredictSigmaPoints(const MatrixXd& Xsig_aug, double delta_t)
{
   // predict sigma points
  for (int i = 0; i < 2 * n_aug_ + 1; ++i) 
  {
    // extract values for better readability
    double p_x = Xsig_aug(0,i);
    double p_y = Xsig_aug(1,i);
    double v = Xsig_aug(2,i);
    double yaw = Xsig_aug(3,i);
    double yawd = Xsig_aug(4,i);
    double nu_a = Xsig_aug(5,i);
    double nu_yawdd = Xsig_aug(6,i);

    // predicted state values
    double px_p, py_p;

    // avoid division by zero
    if (fabs(yawd) > 0.001) 
    {
        px_p = p_x + v/yawd * ( sin(yaw + yawd * delta_t) - sin(yaw));
        py_p = p_y + v/yawd * ( cos(yaw) - cos(yaw + yawd * delta_t) );
    } 
    else
    {
        px_p = p_x + v * delta_t * cos(yaw);
        py_p = p_y + v * delta_t * sin(yaw);
    }

    double v_p = v;
    double yaw_p = yaw + yawd * delta_t;
    double yawd_p = yawd;

    // add noise
    px_p = px_p + 0.5 * nu_a * delta_t * delta_t * cos(yaw);
    py_p = py_p + 0.5 * nu_a * delta_t * delta_t * sin(yaw);
    v_p = v_p + nu_a * delta_t;

    yaw_p = yaw_p + 0.5 * nu_yawdd * delta_t * delta_t;
    yawd_p = yawd_p + nu_yawdd * delta_t;

    // write predicted sigma point into right column
    Xsig_pred_(0,i) = px_p;
    Xsig_pred_(1,i) = py_p;
    Xsig_pred_(2,i) = v_p;
    Xsig_pred_(3,i) = yaw_p;
    Xsig_pred_(4,i) = yawd_p;
  }
}


void UKF::UpdateLidar(MeasurementPackage meas_package) 
{
  /**
   * TODO: Complete this function! Use lidar data to update the belief 
   * about the object's position. Modify the state vector, x_, and 
   * covariance, P_.
   * You can also calculate the lidar NIS, if desired.
   */

  // set measurement dimension, lidar can measure x and y
  int n_z = 2;
  // create matrix for sigma points in measurement space
  MatrixXd Zsig = MatrixXd(n_z, 2 * n_aug_ + 1);
  // mean predicted measurement
  VectorXd z_pred = VectorXd(n_z);
  // measurement covariance matrix S
  MatrixXd S = MatrixXd(n_z, n_z);

  // Measurement noise
  MatrixXd R = MatrixXd(n_z, n_z);
  R.fill(0.0);
  R(0, 0) = std_laspx_ * std_laspx_;
  R(1, 1) = std_laspy_ * std_laspy_;

  // transform sigma points into measurement space
  for(auto i = 0; i < Xsig_pred_.cols(); i++)
  {
    Zsig(0, i) = Xsig_pred_(0, i);
    Zsig(1, i) = Xsig_pred_(1, i);
  }

  z_pred.fill(0.0);
  // calculate mean predicted measurement
  for(auto i = 0; i < Zsig.cols(); i++)
  {
    z_pred += weights_(i) * Zsig.col(i);
  }

  S.fill(0.0);
  // calculate innovation covariance matrix S
  for(auto i = 0; i < Zsig.cols(); i++)
  {
    VectorXd z_diff = Zsig.col(i) - z_pred;
    S += weights_(i) * z_diff * z_diff.transpose();
  }
  // Add measurement noise to covariance matrix
  S = S + R;

  // create matrix for cross correlation Tc
  MatrixXd Tc = MatrixXd(n_x_, n_z);

  // calculate cross correlation matrix
  Tc.fill(0.0);
  for(auto i = 0; i < Zsig.cols(); i++)
  {
    VectorXd x_diff = Xsig_pred_.col(i) - x_;
    VectorXd z_diff = Zsig.col(i) - z_pred;

    // angle normalization
    while (x_diff(3) > M_PI) 
      x_diff(3) -= 2.0 * M_PI;
    while (x_diff(3) < -M_PI) 
      x_diff(3) += 2.0 * M_PI;

    Tc += weights_(i) * x_diff * z_diff.transpose();
  }

  // calculate Kalman gain K;
  MatrixXd K = Tc * S.inverse();
  // update state mean and covariance matrix
  // lidar measurement
  VectorXd z = meas_package.raw_measurements_;
  // residual
  VectorXd z_diff = z - z_pred;

  // update state mean and covariance matrix
  x_ = x_ + K * z_diff;
  P_ = P_ - K * S * K.transpose();
  // Calculate NIS
  NIS_lidar_ = z_diff.transpose() * S.inverse() * z_diff;
  std::cout << "Lidar NIS " << NIS_lidar_ << std::endl;
}

void UKF::UpdateRadar(MeasurementPackage meas_package) 
{
  /**
   * TODO: Complete this function! Use radar data to update the belief 
   * about the object's position. Modify the state vector, x_, and 
   * covariance, P_.
   * You can also calculate the radar NIS, if desired.
   */
  // set measurement dimension, radar can measure r, phi, and r_dot
  int n_z = 3;
  // create matrix for sigma points in measurement space
  MatrixXd Zsig = MatrixXd(n_z, 2 * n_aug_ + 1);
  // mean predicted measurement
  VectorXd z_pred = VectorXd(n_z);
  // measurement covariance matrix S
  MatrixXd S = MatrixXd(n_z,n_z);

  // Measurement noise
  MatrixXd R = MatrixXd(n_z, n_z);
  R.fill(0.0);
  R(0, 0) = std_radr_ * std_radr_;
  R(1, 1) = std_radphi_ * std_radphi_;
  R(2, 2) = std_radrd_ * std_radrd_;
  
  z_pred.fill(0.0);
  S.fill(0.0);
  // transform sigma points into measurement space
  for(auto i = 0; i < Xsig_pred_.cols(); i++)
  {
    auto px  = Xsig_pred_(0, i);
    auto py  = Xsig_pred_(1, i);
    auto v   = Xsig_pred_(2, i);
    auto psi = Xsig_pred_(3, i);

    auto range = sqrt(px * px + py * py);
    auto phi = atan2(py, px);
    auto delta_phi = (px * cos(psi) * v + py * sin(psi) * v) / range;

    Zsig(0, i) = range;
    Zsig(1, i) = phi;
    Zsig(2, i) = delta_phi;
  }

  z_pred.fill(0.0);
  // calculate mean predicted measurement
  for(auto i = 0; i < Zsig.cols(); i++)
  {
    z_pred += weights_(i) * Zsig.col(i);
  }

  // calculate innovation covariance matrix S
  for(auto i = 0; i < Zsig.cols(); i++)
  {
    VectorXd z_diff = Zsig.col(i) - z_pred;
    // angle normalization
    while (z_diff(1) > M_PI) 
      z_diff(1) -= 2.0 * M_PI;
    while (z_diff(1) < -M_PI) 
      z_diff(1) += 2.0 * M_PI;

    S += weights_(i) * z_diff * z_diff.transpose();
  }

  S = S + R;

  // create matrix for cross correlation Tc
  MatrixXd Tc = MatrixXd(n_x_, n_z);
  // calculate cross correlation matrix
  Tc.fill(0.0);
  for(auto i = 0; i < Zsig.cols(); i++)
  {
    VectorXd x_diff = Xsig_pred_.col(i) - x_;
    VectorXd z_diff = Zsig.col(i) - z_pred;
    // angle normalization
    while (z_diff(1) > M_PI) 
      z_diff(1) -= 2.0 * M_PI;
    while (z_diff(1) < -M_PI) 
      z_diff(1) += 2.0 * M_PI;
    // angle normalization
    while (x_diff(3) > M_PI) 
      x_diff(3) -= 2.0 * M_PI;
    while (x_diff(3) < -M_PI) 
      x_diff(3) += 2.0 * M_PI;

    Tc += weights_(i) * x_diff * z_diff.transpose();
  }  

  // calculate Kalman gain K;
  MatrixXd K = Tc * S.inverse();
  // update state mean and covariance matrix
  // residual
  VectorXd z = meas_package.raw_measurements_;
  VectorXd z_diff = z - z_pred;  
  // angle normalization
  while (z_diff(1) > M_PI)
   z_diff(1) -= 2.0 * M_PI;
  while (z_diff(1) < -M_PI)
   z_diff(1) += 2.0 * M_PI;

  x_ = x_ + K * z_diff;
  P_ = P_ - K * S * K.transpose();

  // Calculate NIS for radar
  NIS_radar_ = z_diff.transpose() * S.inverse() * z_diff;
  std::cout << "Radar NIS " << NIS_radar_ << std::endl;

}