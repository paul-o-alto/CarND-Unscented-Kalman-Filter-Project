#include "ukf.h"
#include "Eigen/Dense"
#include <iostream>

using namespace std;
using Eigen::MatrixXd;
using Eigen::VectorXd;
using std::vector;

//using namespace std;

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
  std_a_ = 15; //30; //0.4;

  // Process noise standard deviation yaw acceleration in rad/s^2
  std_yawdd_ = 6; //30; //0.2;

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

  // previous timestamp
  time_us_ = 0;

  /**
  TODO:

  Complete the initialization. See ukf.h for other member properties.

  Hint: one or more values initialized above might be wildly off...
  */

  // initializing matrices
  this->Xsig_pred_ = MatrixXd(11, 5);

  this->F_ = MatrixXd(4, 4);
  this->F_ << 1, 0, 1, 0,
              0, 1, 0, 1,
              0, 0, 1, 0,
              0, 0, 0, 1;

  //this->Q_ = MatrixXd(4, 4);


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
  if (!is_initialized_) {

    //cout << "Initialize the state this.x_ with the first measurement." << endl;
    

    /* Create the covariance matrix. */
    this->P_ << 10, 0, 0, 0, 0,
                0, 10, 0, 0, 0,
                0, 0, 10, 0, 0,
                0, 0, 0, 10, 0,
                0, 0, 0, 0, 10;


    // first measurement
    std::cout << "UKF: " << std::endl;
   
    if (meas_package.sensor_type_ == MeasurementPackage::RADAR) {    
      VectorXd polar(3);
      polar << meas_package.raw_measurements_[0], meas_package.raw_measurements_[1], meas_package.raw_measurements_[2];
      std::cout << "Converting polar to cartesian and Initializing state" << std::endl;
      this->x_ << polar[0]*cos(polar[1]), polar[0]*sin(polar[1]), 0, 0, 0;
    }
    else if (meas_package.sensor_type_ == MeasurementPackage::LASER) {
      std::cout << "Initializing state." << std::endl;
      this->x_ << meas_package.raw_measurements_[0], meas_package.raw_measurements_[1], 0, 0, 0;
    }

    cout << "Done initializing, no need to predict or update" << endl;
    this->time_us_ = meas_package.timestamp_;
    is_initialized_ = true;
    return;

  }

  float dt = (meas_package.timestamp_ - this->time_us_) / 1000000.0;   //dt - expressed in seconds
  //std::cout << dt << std::endl;
  time_us_ = meas_package.timestamp_;

  this->F_(0, 2) = dt;
  this->F_(1, 3) = dt;

  float dt_2 = dt * dt;
  float dt_3 = dt_2 * dt;
  float dt_4 = dt_3 * dt;


  //std::cout << "Performing prediction step" << std::endl;
  this->Prediction(dt);

  if (meas_package.sensor_type_ == MeasurementPackage::RADAR && use_radar_) {
    // Radar updates
    //std::cout << "Processing Radar measurement" << std::endl;
    this->UpdateRadar(meas_package); //.raw_measurements_);
  } else if (meas_package.sensor_type_ == MeasurementPackage::LASER && use_laser_) {
    // Laser updates
    //std::cout << "Processing Laser measurement" << std::endl;
    this->UpdateLidar(meas_package); //.raw_measurements_);
  }

  // print the output
  //std::cout << "x_ = " << this->x_ << std::endl;
  //std::cout << "P_ = " << this->P_ << std::endl;

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

  MatrixXd Xsig_out;

  //cout << "Generate sigma points" << endl;
  this->AugmentedSigmaPoints(&Xsig_out);

  //cout << "Predict sigma points" << endl;
  this->SigmaPointPrediction(delta_t, &Xsig_out);

  //cout << "Predict mean and covariance" << endl;
  //VectorXd x_pred = VectorXd(5);
  //MatrixXd P_pred = MatrixXd(5, 5);
  this->PredictMeanAndCovariance(); 

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

  //cout << "Predict Lidar measurement" << endl;
  this->PredictMeasurement(2);

  VectorXd z(2);
  //cout << "Getting Lidar measurement for Update step." << std::endl;
  z << meas_package.raw_measurements_[0], meas_package.raw_measurements_[1];

  //cout << "Update state" << endl;
  this->UpdateState(z, 2);

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

  //cout << "Predict Radar measurement" << endl;
  this->PredictMeasurement(3);

  VectorXd polar(3);
  polar << meas_package.raw_measurements_[0], meas_package.raw_measurements_[1], meas_package.raw_measurements_[2];

  //cout << "Update state" << endl;
  this->UpdateState(polar, 3);

}

void UKF::GenerateSigmaPoints(MatrixXd* Xsig_out) {

  //set state dimension
  int n_x = 5;

  //define spreading parameter
  double lambda = 3 - n_x;

  //create sigma point matrix
  MatrixXd Xsig_ = MatrixXd(n_x, 2 * n_x + 1);

  //calculate square root of P
  MatrixXd A = P_.llt().matrixL();

  //set first column of sigma point matrix
  Xsig_.col(0)  = this->x_;

  //set remaining sigma points
  for (int i = 0; i < n_x; i++)
  {
    Xsig_.col(i+1)     = this->x_ + sqrt(lambda+n_x) * A.col(i);
    Xsig_.col(i+1+n_x) = this->x_ - sqrt(lambda+n_x) * A.col(i);
  }

  //print result
  //std::cout << "Xsig = " << std::endl << Xsig_ << std::endl;
  //write result
  *Xsig_out = Xsig_;

}

void UKF::AugmentedSigmaPoints(MatrixXd* Xsig_out) {

  //set state dimension
  int n_x = 5;

  //set augmented dimension
  int n_aug = 7;

  //define spreading parameter
  double lambda = 3 - n_aug;

  //create augmented mean vector
  VectorXd x_aug = VectorXd(7);

  //create augmented state covariance
  MatrixXd P_aug = MatrixXd(7, 7);

  //create sigma point matrix
  MatrixXd Xsig_aug = MatrixXd(n_aug, 2 * n_aug + 1);

  //create augmented mean state
  x_aug.head(5) = this->x_;
  x_aug(5) = 0;
  x_aug(6) = 0;

  //create augmented covariance matrix
  P_aug.fill(0.0);
  P_aug.topLeftCorner(5,5) = this->P_;
  P_aug(5,5) = this->std_a_*this->std_a_;
  P_aug(6,6) = this->std_yawdd_*this->std_yawdd_;

  //create square root matrix
  MatrixXd L = P_aug.llt().matrixL();

  //create augmented sigma points
  Xsig_aug.col(0)  = x_aug;
  for (int i = 0; i< n_aug; i++)
  {
    Xsig_aug.col(i+1)       = x_aug + sqrt(lambda+n_aug) * L.col(i);
    Xsig_aug.col(i+1+n_aug) = x_aug - sqrt(lambda+n_aug) * L.col(i);
  }
  
  //print result
  //std::cout << "Xsig_aug = " << std::endl << Xsig_aug << std::endl;

  //write result
  *Xsig_out = Xsig_aug;
}

void UKF::SigmaPointPrediction(double delta_t, MatrixXd* Xsig_in) {

  //set state dimension
  int n_x = 5;

  //set augmented dimension
  int n_aug = 7;

  //create matrix with predicted sigma points as columns
  MatrixXd Xsig_pred_ = MatrixXd(n_x, 2 * n_aug + 1);

  MatrixXd Xsig_aug = *Xsig_in;

  //predict sigma points
  for (int i = 0; i< 2*n_aug+1; i++)
  {
    //extract values for better readability
    double p_x = Xsig_aug(0,i);
    double p_y = Xsig_aug(1,i);
    double v = Xsig_aug(2,i);
    double yaw = Xsig_aug(3,i);
    double yawd = Xsig_aug(4,i);
    double nu_a = Xsig_aug(5,i);
    double nu_yawdd = Xsig_aug(6,i);

    //predicted state values
    double px_p, py_p;

    //avoid division by zero
    if (abs(yawd) > 0.001) {
        px_p = p_x + v/yawd * ( sin (yaw + yawd*delta_t) - sin(yaw));
        py_p = p_y + v/yawd * ( cos(yaw) - cos(yaw+yawd*delta_t) );
    }
    else {
        px_p = p_x + v*delta_t*cos(yaw);
        py_p = p_y + v*delta_t*sin(yaw);
    }

    double v_p = v;
    double yaw_p = yaw + yawd*delta_t;
    double yawd_p = yawd;

    //add noise
    px_p = px_p + 0.5*nu_a*delta_t*delta_t * cos(yaw);
    py_p = py_p + 0.5*nu_a*delta_t*delta_t * sin(yaw);
    v_p = v_p + nu_a*delta_t;

    yaw_p = yaw_p + 0.5*nu_yawdd*delta_t*delta_t;
    yawd_p = yawd_p + nu_yawdd*delta_t;

    //write predicted sigma point into right column
    Xsig_pred_(0,i) = px_p;
    Xsig_pred_(1,i) = py_p;
    Xsig_pred_(2,i) = v_p;
    Xsig_pred_(3,i) = yaw_p;
    Xsig_pred_(4,i) = yawd_p;
  }

  //print result
  //std::cout << "Xsig_pred = " << std::endl << Xsig_pred_ << std::endl;

  this->Xsig_pred_ = Xsig_pred_;

}

void UKF::PredictMeanAndCovariance() {

  //set state dimension
  int n_x = 5;

  //set augmented dimension
  int n_aug = 7;

  //define spreading parameter
  double lambda = 3 - n_aug;

  //create example matrix with predicted sigma points
  //Xsig_pred_ = MatrixXd(n_x, 2 * n_aug + 1);

  //create vector for weights
  VectorXd weights = VectorXd(2*n_aug+1);
  
  // set weights
  double weight_0 = lambda/(lambda+n_aug);
  weights(0) = weight_0;
  for (int i=1; i<2*n_aug+1; i++) {  //2n+1 weights
    double weight = 0.5/(n_aug+lambda);
    weights(i) = weight;
  }

  //predicted state mean
  this->x_.fill(0.0);
  for (int i = 0; i < 2 * n_aug + 1; i++) {  //iterate over sigma points
    this->x_ = this->x_ + weights(i) * this->Xsig_pred_.col(i);
  }

  //predicted state covariance matrix
  this->P_.fill(0.0);
  for (int i = 0; i < 2 * n_aug + 1; i++) {  //iterate over sigma points

    // state difference
    VectorXd x_diff = this->Xsig_pred_.col(i) - this->x_;
    //angle normalization
    while (x_diff(3)> M_PI) x_diff(3)-=2.*M_PI;
    while (x_diff(3)<-M_PI) x_diff(3)+=2.*M_PI;

    this->P_ = this->P_ + weights(i) * x_diff * x_diff.transpose() ;
  }

  //print result
  //std::cout << "Predicted state" << std::endl;
  //std::cout << this->x_ << std::endl;
  //std::cout << "Predicted covariance matrix" << std::endl;
  //std::cout << this->P_ << std::endl;

  Zsig_ = Xsig_pred_;
  //z_pred_ = this->x_;
}

void UKF::PredictMeasurement(int n_z) {

  //set state dimension
  int n_x = 5;

  //set augmented dimension
  int n_aug = 7;

  //set measurement dimension, radar can measure r, phi, and r_dot
  //int n_z = 3;

  //define spreading parameter
  double lambda = 3 - n_aug;

  //set vector for weights
  VectorXd weights = VectorXd(2*n_aug+1);
   double weight_0 = lambda/(lambda+n_aug);
  weights(0) = weight_0;
  for (int i=1; i<2*n_aug+1; i++) {  
    double weight = 0.5/(n_aug+lambda);
    weights(i) = weight;
  }


  //create matrix for sigma points in measurement space
  Zsig_ = MatrixXd(n_z, 2 * n_aug + 1);

  //transform sigma points into measurement space
  for (int i = 0; i < 2 * n_aug + 1; i++) {  //2n+1 simga points

    // extract values for better readibility
    double p_x = Xsig_pred_(0,i);
    double p_y = Xsig_pred_(1,i);
    double v  = Xsig_pred_(2,i);
    double yaw = Xsig_pred_(3,i);

    double v1 = cos(yaw)*v;
    double v2 = sin(yaw)*v;

    // measurement model
    if (n_z == 3) {
        Zsig_(0,i) = sqrt(p_x*p_x + p_y*p_y);                        //r
        Zsig_(1,i) = atan2(p_y,p_x);                                 //phi
        Zsig_(2,i) = (p_x*v1 + p_y*v2 ) / sqrt(p_x*p_x + p_y*p_y);   //r_dot
    } else if (n_z == 2) {
        Zsig_(0,i) = p_x;
        Zsig_(1,i) = p_y;
    }
  }

  //mean predicted measurement
  z_pred_ = VectorXd(n_z);
  z_pred_.fill(0.0);
  for (int i=0; i < 2*n_aug+1; i++) {
      z_pred_ = z_pred_ + weights(i) * Zsig_.col(i);
  }

  //measurement covariance matrix S
  MatrixXd S = MatrixXd(n_z,n_z);
  S.fill(0.0);
  for (int i = 0; i < 2 * n_aug + 1; i++) {  //2n+1 simga points
    //residual
    VectorXd z_diff = Zsig_.col(i) - z_pred_;

    //angle normalization
    while (z_diff(1)> M_PI) z_diff(1)-=2.*M_PI;
    while (z_diff(1)<-M_PI) z_diff(1)+=2.*M_PI;

    S = S + weights(i) * z_diff * z_diff.transpose();
  }

  //add measurement noise covariance matrix
  MatrixXd R = MatrixXd(n_z,n_z);
  if (n_z == 3) {
    R <<  this->std_radr_*this->std_radr_, 0, 0,
          0, this->std_radphi_*this->std_radphi_, 0,
          0, 0, this->std_radrd_*this->std_radrd_;
  } else if (n_z == 2) {
    R << this->std_laspx_*this->std_laspx_, 0,
         0, this->std_laspy_*this->std_laspy_;
  }
  S = S + R;

  //print result
  //std::cout << "z_pred: " << std::endl << z_pred_ << std::endl;
  //std::cout << "S: " << std::endl << S << std::endl;

  S_ = S;
}

void UKF::UpdateState(VectorXd z, int n_z) {


  //set state dimension
  int n_x = 5;

  //set augmented dimension
  int n_aug = 7;

  //define spreading parameter
  double lambda = 3 - n_aug;

  //cout << "Set vector for weights" << endl;
  VectorXd weights = VectorXd(2*n_aug+1);
   double weight_0 = lambda/(lambda+n_aug);
  weights(0) = weight_0;
  for (int i=1; i<2*n_aug+1; i++) {  //2n+1 weights
    double weight = 0.5/(n_aug+lambda);
    weights(i) = weight;
  }


  //create matrix for cross correlation Tc
  MatrixXd Tc = MatrixXd(n_x, n_z);

  //cout << "Calculating cross correlation matrix" << endl;
  Tc.fill(0.0);
  for (int i = 0; i < 2 * n_aug + 1; i++) {  //2n+1 simga points

    //residual
    VectorXd z_diff = Zsig_.col(i) - z_pred_;
    //angle normalization
    while (z_diff(1)> M_PI) z_diff(1)-=2.*M_PI;
    while (z_diff(1)<-M_PI) z_diff(1)+=2.*M_PI;

    // state difference
    VectorXd x_diff = Xsig_pred_.col(i) - this->x_;
    //angle normalization
    while (x_diff(3)> M_PI) x_diff(3)-=2.*M_PI;
    while (x_diff(3)<-M_PI) x_diff(3)+=2.*M_PI;

    Tc = Tc + weights(i) * x_diff * z_diff.transpose();
  }

  //cout << "Getting Kalman gain" << endl;
  MatrixXd K = Tc * S_.inverse();

  //cout << "residual: z size: " << z.size() << ", z_pred_ size" << z_pred_.size() << endl;
  VectorXd z_diff = z - z_pred_;

  //angle normalization
  while (z_diff(1)> M_PI) z_diff(1)-=2.*M_PI;
  while (z_diff(1)<-M_PI) z_diff(1)+=2.*M_PI;

  //NIS Calculation
  VectorXd diff = Zsig_.col(0) - z_pred_;
  VectorXd temp = diff.transpose()*S_.inverse();
  double nis = temp.dot(diff);
  cout << nis << endl;
  if (n_z == 3) {
      NIS_radar_ = nis;
  } else if (n_z == 2) {
      NIS_laser_ = nis;
  }


  //cout << "Updating state mean" << endl;
  this->x_ = this->x_ + K * z_diff;
  //cout << "Updating state covariance" << endl;
  this->P_ = this->P_ - K*S_*K.transpose();

  //print result
  //std::cout << "Updated state x: " << std::endl << this->x_ << std::endl;
  //std::cout << "Updated state covariance P: " << std::endl << this->P_ << std::endl;

}
