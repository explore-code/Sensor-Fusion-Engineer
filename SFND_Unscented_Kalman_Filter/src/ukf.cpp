#include "ukf.h"
#include "Eigen/Dense"
#include <iostream>


using Eigen::MatrixXd;
using Eigen::VectorXd;

const bool DEBUG = false;
const bool DEBUG_INFO = false;

/**
 * Initializes Unscented Kalman filter
 */
UKF::UKF() {

  // if this is false, laser measurements will be ignored (except during init)
  use_laser_ = true;

  // if this is false, radar measurements will be ignored (except during init)
  use_radar_ = true;

  // initial state vector
  x_ = VectorXd(5);

  // initial covariance matrix
  P_ = MatrixXd(5, 5);

  // Process noise standard deviation longitudinal acceleration in m/s^2
  std_a_ = 1.45; // Max acceleration is 3.0

  // Process noise standard deviation yaw acceleration in rad/s^2
  std_yawdd_ = 0.45; // Max steering is +/-1.0
  
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

  is_initialized_ = false;

  // [ pos1 pos2 vel_abs yaw_angle yaw_rate ]
  n_x_ = 5;

  // [ pos1 pos2 vel_abs yaw_angle yaw_rate v_a, v_yaw_dd ]
  n_aug_ = 7;

  // Spreading parameter
  lambda_ = 3 - n_aug_;

  // Augmented weights
  weights_ = VectorXd(2 * n_aug_ + 1);
  weights_(0) = lambda_ / (lambda_ + n_aug_);
  for(int i = 1; i < 2 * n_aug_ + 1; ++i)
  {
    weights_(i) = 1 / (2 * (lambda_ + n_aug_));
  }

  // Predicted sigma points
  Xsig_pred_ = MatrixXd(n_x_, 2 * n_aug_ + 1);

  // Time of the Kalman filter
  time_us_ = 0;

}

UKF::~UKF() {}

void UKF::ProcessMeasurement(MeasurementPackage meas_package) {
  /**
   * TODO: Complete this function! Make sure you switch between lidar and radar
   * measurements.
   */

  if(DEBUG)
  {
    std::cout << "New measurement data package" << std::endl;
    std::cout << "----------------------------" << std::endl;
    std::cout << "Timestamp = " << meas_package.timestamp_ << std::endl;
    std::cout << "Raw measurement = " << meas_package.raw_measurements_ << std::endl;
    std::cout << std::endl;
  }

  if(is_initialized_)
  {

    const long long t = meas_package.timestamp_;

    if(meas_package.sensor_type_ == MeasurementPackage::LASER && use_laser_)
    {

      const double delta_t = (t - time_us_) / 1e6;

      if(DEBUG_INFO)
      {
        std::cout << "delta_ t = " << delta_t << std::endl;
      }

      if(delta_t > 0.0)
      {
        Prediction(delta_t);
      }

      UpdateLidar(meas_package);

      // Update the time
      time_us_ = t;

    }

    if(meas_package.sensor_type_ == MeasurementPackage::RADAR && use_radar_)
    {

      const double delta_t = (t - time_us_) / 1e6;

      if(DEBUG_INFO)
      {
        std::cout << "delta_ t = " << delta_t << std::endl;
      }

      if(delta_t > 0.0)
      {
        Prediction(delta_t);
      }

      UpdateRadar(meas_package);

      // Update the time
      time_us_ = t;

    }

  }
  else
  {

    if(meas_package.sensor_type_ == MeasurementPackage::LASER && use_laser_)
    {

      const double px = meas_package.raw_measurements_(0);
      const double py = meas_package.raw_measurements_(1);

      x_ << px,
            py,
            0,
            0,
            0;

      if(DEBUG_INFO)
      {
        std::cout << "Laser initialization performed: " << std::endl << x_ << std::endl;
      }

      P_ << 0.1, 0, 0, 0, 0,
            0, 0.1, 0, 0, 0,
            0, 0, 1, 0, 0,
            0, 0, 0, 1, 0,
            0, 0, 0, 0, 1;

      Prediction(0);

      time_us_ = meas_package.timestamp_;

      is_initialized_ = true;

    }

    if(meas_package.sensor_type_ == MeasurementPackage::RADAR && use_radar_)
    {

      const double rho = meas_package.raw_measurements_(0);
      const double phi = meas_package.raw_measurements_(1);
      const double rho_d = meas_package.raw_measurements_(2);      

      x_ << rho * cos(phi),
            rho * sin(phi),
            0,
            0,
            0;

      if(DEBUG_INFO)
      {
        std::cout << "Radar initialization performed: " << std::endl << x_ << std::endl;
      }

      P_ << 0.25, 0, 0, 0, 0,
            0, 0.25, 0, 0, 0,
            0, 0, 1, 0, 0,
            0, 0, 0, 1, 0,
            0, 0, 0, 0, 1;

      Prediction(0);

      time_us_ = meas_package.timestamp_;
    
      is_initialized_ = true;

    }

  }
  
}

void UKF::Prediction(double delta_t) {

  /**
   * TODO: Complete this function! Estimate the object's location. 
   * Modify the state vector, x_. Predict sigma points, the state, 
   * and the state covariance matrix.
   */

  // Augmented state mean
  VectorXd x_aug = VectorXd::Zero(n_aug_);
  x_aug.head(n_x_) = x_;

  // Augmented state covariance matrix
  MatrixXd P_aug = MatrixXd::Zero(n_aug_, n_aug_);
  P_aug.topLeftCorner(n_x_, n_x_) = P_;
  P_aug(5, 5) = std_a_ * std_a_;
  P_aug(6, 6) = std_yawdd_ * std_yawdd_;

  // Create square root matrix
  MatrixXd A_aug = P_aug.llt().matrixL();

  // Create augmented sigma points
  MatrixXd Xsig_aug = MatrixXd(n_aug_, 2 * n_aug_ + 1);

  Xsig_aug.col(0) = x_aug;
  for(int i = 1; i <= n_aug_; ++i)
  {
    Xsig_aug.col(i) = x_aug + sqrt(lambda_ + n_aug_) * A_aug.col(i - 1);
    Xsig_aug.col(i + n_aug_) = x_aug - sqrt(lambda_ + n_aug_) * A_aug.col(i - 1);
  }

  for(int i = 0; i < 2 * n_aug_ + 1; ++i)
  {

    if(DEBUG)
    {
      std::cerr << "DEBUG 1.25" << std::endl;
    }

    const double px = Xsig_aug(0, i);
    const double py = Xsig_aug(1, i);
    const double v = Xsig_aug(2, i);
    const double yaw = Xsig_aug(3, i);
    const double yaw_d = Xsig_aug(4, i);
    const double v_a = Xsig_aug(5, i);
    const double v_yaw_dd = Xsig_aug(6, i);

    Xsig_pred_.col(i) = Xsig_aug.col(i).head(n_x_);

    if(DEBUG)
    {
      std::cerr << "DEBUG 1 3/8" << std::endl;
    }

    if(fabs(yaw_d) <= 1e-4)
    {
      Xsig_pred_(0, i) += v * cos(yaw) * delta_t;
      Xsig_pred_(1, i) += v * sin(yaw) * delta_t;
    }
    else
    {
      Xsig_pred_(0, i) += v / yaw_d * (sin(yaw + yaw_d * delta_t) - sin(yaw));
      Xsig_pred_(1, i) += v / yaw_d * (-cos(yaw + yaw_d * delta_t) + cos(yaw));
    }

    if(DEBUG)
    {
      std::cerr << "DEBUG 1.5" << std::endl;
    }
    
    Xsig_pred_(2, i) += 0;
    Xsig_pred_(3, i) += yaw_d * delta_t;
    Xsig_pred_(4, i) += 0;

    // Add noise
    VectorXd noise(n_x_);
    noise << 0.5 * delta_t * delta_t * cos(yaw) * v_a,
             0.5 * delta_t * delta_t * sin(yaw) * v_a,
             delta_t * v_a,
             0.5 * delta_t * delta_t * v_yaw_dd,
             delta_t * v_yaw_dd;

    Xsig_pred_.col(i) += noise;

  }

  // Update state mean
  x_ = VectorXd::Zero(n_x_);
  for(int i = 0; i < 2 * n_aug_ + 1; ++i)
  {
    x_ += weights_(i) * Xsig_pred_.col(i);
  }
  normalizeYaw(x_(3));

  // Update state covariance
  P_ = MatrixXd::Zero(n_x_, n_x_);
  for(int i = 0; i < 2 * n_aug_ + 1; ++i)
  {
    VectorXd x_diff = Xsig_pred_.col(i) - x_;
    normalizeYaw(x_diff(3));

    P_ += weights_(i) * x_diff * x_diff.transpose();
  }

}

void UKF::UpdateLidar(MeasurementPackage meas_package) {

  /**
   * TODO: Complete this function! Use lidar data to update the belief 
   * about the object's position. Modify the state vector, x_, and 
   * covariance, P_.
   * You can also calculate the lidar NIS, if desired.
   */

  VectorXd z = meas_package.raw_measurements_;

  // Dimension of lidar measurement
  const int n_z = 2;

  // Sigma points 
  MatrixXd Zsig = MatrixXd::Zero(n_z, 2 * n_aug_ + 1);

  // Transform sigma points into measurement space
  for(int i = 0; i < 2 * n_aug_ + 1; ++i)
  {

      const double px = Xsig_pred_(0, i);
      const double py = Xsig_pred_(1, i);

      Zsig(0, i) = px;
      Zsig(1, i) = py;

  }

  // Calculate mean predicted measurement
  VectorXd z_pred = VectorXd::Zero(n_z);
  calculateMeanPredictedMeasurement(Zsig, z_pred);  

  // Calculate innovation covariance matrix S
  MatrixXd R(n_z, n_z);
  R << std_laspx_ * std_laspx_, 0,
       0, std_laspy_ * std_laspy_;
  MatrixXd S = MatrixXd::Zero(n_z, n_z);
  calculateInnovationMatrixLidar(S, Zsig, z_pred, R);

  // Cross correlation matrix
  MatrixXd T = MatrixXd::Zero(n_x_, n_z);
  calculateCrossCorrelationMatrixLidar(T, n_z, Zsig, z_pred);

  // Kalman gain
  MatrixXd K = T * S.inverse();

  // Residual
  VectorXd z_diff_mp = z - z_pred;

  // Update state mean and covariance
  updateMeanAndCovarianceWithMeasurement(K, S, z_diff_mp);

}

void UKF::UpdateRadar(MeasurementPackage meas_package) {

  /**
   * TODO: Complete this function! Use radar data to update the belief 
   * about the object's position. Modify the state vector, x_, and 
   * covariance, P_.
   * You can also calculate the radar NIS, if desired.
   */

  VectorXd z = meas_package.raw_measurements_;

  // Dimension of lidar measurement
  const int n_z = 3;

  // Sigma points 
  MatrixXd Zsig = MatrixXd::Zero(n_z, 2 * n_aug_ + 1);

  // Transform sigma points into measurement space
  for(int i = 0; i < 2 * n_aug_ + 1; ++i)
  {

      const double px = Xsig_pred_(0, i);
      const double py = Xsig_pred_(1, i);
      const double v = Xsig_pred_(2, i);
      const double yaw = Xsig_pred_(3, i);

      Zsig(0, i) = sqrt(px * px + py * py);
      Zsig(1, i) = atan2(py, px);
      Zsig(2, i) = (px * cos(yaw) * v + py * sin(yaw) * v) / Zsig(0, i);

  }

  // Calculate mean predicted measurement
  VectorXd z_pred = VectorXd::Zero(n_z);
  calculateMeanPredictedMeasurement(Zsig, z_pred);  

  // Calculate innovation covariance matrix S
  MatrixXd R(n_z, n_z);
  R << std_radr_ * std_radr_, 0, 0,
       0, std_radphi_ * std_radphi_, 0,
       0, 0, std_radrd_ * std_radrd_;
  // Measurement covariance
  MatrixXd S = MatrixXd::Zero(n_z, n_z);
  calculateInnovationMatrixRadar(S, Zsig, z_pred, R);  

  // Cross correlation matrix
  MatrixXd T = MatrixXd::Zero(n_x_, n_z);
  calculateCrossCorrelationMatrixRadar(T, n_z, Zsig, z_pred);

  // Kalman gain
  MatrixXd K = T * S.inverse();

  // Residual
  VectorXd z_diff_mp = z - z_pred;
  normalizeYaw(z_diff_mp(1));

  // Update state mean and covariance
  updateMeanAndCovarianceWithMeasurement(K, S, z_diff_mp);

}

void UKF::calculateMeanPredictedMeasurement(const MatrixXd& Zsig, VectorXd& z_pred)
{

  for(int i = 0; i < 2 * n_aug_ + 1; ++i)
  {
    z_pred += weights_(i) * Zsig.col(i);
  }

}

void UKF::calculateInnovationMatrixLidar(MatrixXd& S, const MatrixXd& Zsig, const VectorXd& z_pred, const MatrixXd& R)
{

  for(int i = 0; i < 2 * n_aug_ + 1; ++i)
  {
    const VectorXd v = Zsig.col(i) - z_pred;
    S += weights_(i) * v * v.transpose();
  }

  S += R;

}

void UKF::calculateInnovationMatrixRadar(MatrixXd& S, const MatrixXd& Zsig, const VectorXd& z_pred, const MatrixXd& R)
{

  for(int i = 0; i < 2 * n_aug_ + 1; ++i)
  {

    VectorXd z_diff = Zsig.col(i) - z_pred;
    normalizeYaw(z_diff(1));   

    S += weights_(i) * z_diff * z_diff.transpose();

  }

  S += R;

}

void UKF::calculateCrossCorrelationMatrixLidar(MatrixXd& T, const int n_z, const MatrixXd& Zsig, const VectorXd& z_pred)
{

  for(int i = 0; i < 2 * n_aug_ + 1; ++i)
  {

    VectorXd x_diff = Xsig_pred_.col(i) - x_;
    VectorXd z_diff = Zsig.col(i) - z_pred;    

    T += weights_(i) * x_diff * z_diff.transpose();

  }

}

void UKF::calculateCrossCorrelationMatrixRadar(MatrixXd& T, const int n_z, const MatrixXd& Zsig, const VectorXd& z_pred)
{

  for(int i = 0; i < 2 * n_aug_ + 1; ++i)
  {

    VectorXd z_diff = Zsig.col(i) - z_pred;
    normalizeYaw(z_diff(1));

    VectorXd x_diff = Xsig_pred_.col(i) - x_;
    normalizeYaw(x_diff(3));

    T += weights_(i) * x_diff * z_diff.transpose();

  }

}

void UKF::updateMeanAndCovarianceWithMeasurement(const MatrixXd& K, const MatrixXd& S, const VectorXd& z_diff_mp)
{

  x_ += K * z_diff_mp;
  normalizeYaw(x_(3));

  P_ -= K * S * K.transpose();

  if(DEBUG)
  {
    std::cout << "New state and covariance" << std::endl;
    std::cout << "------------------------" << std::endl;
    std::cout << "State mean: " << std::endl << x_ << std::endl;
    std::cout << "State covariance: " << std::endl << P_ << std::endl;
    std::cout << std::endl;
  }

}

void UKF::normalizeYaw(double& yaw)
{

  double factor = std::floor(std::fabs(yaw) / (2.0 * M_PI));

  if(yaw > 0)
  {
    yaw -= factor * 2.0 * M_PI;
  }
  else
  {
    yaw += factor * 2.0 * M_PI;
  }

}

