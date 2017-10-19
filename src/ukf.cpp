#include "ukf.h"
#include "tools.h"
#include "Eigen/Dense"
#include <iostream>
#include <math.h>

using namespace std;
using Eigen::MatrixXd;
using Eigen::VectorXd;
using std::vector;

/**
 * Initializes Unscented Kalman filter
 */
UKF::UKF() {
	// if this is false, laser measurements will be ignored (except during init)
	use_laser_ = false;

	// if this is false, radar measurements will be ignored (except during init)
	use_radar_ = true;

	n_x_ = 5;
	n_aug_ = 7;
	n_sw_ = 2 * n_aug_ + 1;

	// initial state vector
	x_ = VectorXd(n_x_);

	// initial covariance matrix
	P_ = MatrixXd(n_x_, n_x_);

	weights_ = VectorXd(n_sw_);  //create vector for weights

	Xsig_pred_ = MatrixXd(n_x_, n_sw_);
	Xsig_pred_.fill(0);

	// Process noise standard deviation longitudinal acceleration in m/s^2
	std_a_ = 30;

	// Process noise standard deviation yaw acceleration in rad/s^2
	std_yawdd_ = 30;

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

	is_initialized_ = false;
	time_us_ = 0;
	lambda_ = 3 - n_aug_;
	/**
	TODO:

	Complete the initialization. See ukf.h for other member properties.

	Hint: one or more values initialized above might be wildly off...
	*/
/*	R_radar_ = MatrixXd(3, 3);
	R_radar_ << std_radr_*std_radr_, 0, 0,
		0, std_radphi_*std_radphi_, 0,
		0, 0, std_radrd_*std_radrd_;
	R_lidar_ = MatrixXd(2, 2);
	R_lidar_ << std_laspx_*std_laspx_, 0,
		0, std_laspy_*std_laspy_;
*/
}

UKF::~UKF() {
	
}

/**
 * @param {MeasurementPackage} meas_package The latest measurement data of
 * either radar or laser.
 */
void UKF::ProcessMeasurement(MeasurementPackage meas_package) {
  /** TODO:  Complete this function! Make sure you switch between lidar and radar measurements.*/
	if (!is_initialized_) {
		//init P_ , covariance matrix
		x_.fill(0);
		P_ << 1, 0, 0, 0, 0,
			  0, 1, 0, 0, 0,
			  0, 0, 1, 0, 0,
			  0, 0, 0, 1, 0,
	  		  0, 0, 0, 0, 1;
		
		if (meas_package.sensor_type_ == MeasurementPackage::LASER)//then init x_ using px,py,0,0,0
		{
			double px = meas_package.raw_measurements_[0];
			double py = meas_package.raw_measurements_[1];
			x_(0) = px;
			x_(1) = py;
		}
		else if(meas_package.sensor_type_ == MeasurementPackage::RADAR)//then init x_ using rho,phi,rho_dot,0,0
		{
			float ro = meas_package.raw_measurements_[0];//rho , range
			float theta = meas_package.raw_measurements_[1];//phi , bearing
			//float ro_dot = meas_package.raw_measurements_[2];//rho_dot , rate
			x_(0) = ro * cos(theta);//px;
			x_(1) = ro * sin(theta);//py;
		}
		
//cout << "init x_" << x_ <<"of type" << meas_package.sensor_type_  << endl;
		weights_.fill(0);
		//set weights
		double weight_0 = lambda_ / (lambda_ + n_aug_);
		weights_(0) = weight_0;
		for (int i = 1; i<n_sw_; i++) {  //2n+1 weights
			double weight = 0.5 / (n_aug_ + lambda_);
			weights_(i) = weight;
		}
		
		time_us_ = meas_package.timestamp_;
		is_initialized_ = true;
		return;
	}
	
	float dt = (meas_package.timestamp_ - time_us_) / 1000000.0; //dt - expressed in seconds
	time_us_ = meas_package.timestamp_;
	
	Prediction(dt);
	
	//then TODO: UpdateLidar or UpdateRadar
	if (meas_package.sensor_type_ == MeasurementPackage::RADAR && use_radar_) {
		UpdateRadar(meas_package);
	}
	if (meas_package.sensor_type_ == MeasurementPackage::LASER && use_laser_) {
		UpdateLidar(meas_package);
	}
	
	//avoid division by zero
	//write predicted sigma points into right column
	//print result

	cout << "x_" << endl << x_ << endl;
	cout << "P_" << endl << P_ << endl;
}

/**
 * Predicts sigma points, the state, and the state covariance matrix.
 * @param {double} delta_t the change in time (in seconds) between the last
 * measurement and this one.
 */
void UKF::Prediction(double delta_t) {
  /** TODO:
  Complete this function! Estimate the object's location. Modify the state
  vector, x_. Predict sigma points, the state, and the state covariance matrix.
  */
	VectorXd x_aug = VectorXd(n_aug_);
	//create augmented state covariance
	MatrixXd P_aug = MatrixXd(n_aug_, n_aug_);
	//create sigma point matrix
	MatrixXd Xsig_aug = MatrixXd(n_aug_, n_sw_);
	//create augmented mean state
	x_aug.fill(0);//x_aug << 0,0,0,0,0,0,0;
	x_aug.head(n_x_) = x_;
	//create augmented covariance matrix
	P_aug.topLeftCorner(n_x_, n_x_) = P_;
	MatrixXd Q = MatrixXd(2, 2);
	Q << std_a_*std_a_	,		0,
				0		, std_yawdd_*std_yawdd_;
	P_aug.bottomRightCorner(2, 2) = Q;
	//create square root matrix
	MatrixXd A_aug = P_aug.llt().matrixL();
	MatrixXd A_lambda_aug = MatrixXd(n_aug_, n_aug_);
	A_lambda_aug = A_aug * sqrt(lambda_ + n_aug_);
	//create augmented sigma points
	Xsig_aug.col(0) = x_aug;
	for (int i = 1; i<=n_aug_; i++) {
		Xsig_aug.col(i) = x_aug + A_lambda_aug.col(i - 1);
		Xsig_aug.col(i + n_aug_) = x_aug - A_lambda_aug.col(i - 1);
	}
	//cout << "Xsig_aug" << endl << Xsig_aug << endl;
	//th9is part from assignment answer becuase they handle zero division better than me 
  //Sigma Point Prediction---------------------------
  //create matrix with predicted sigma points as columns
	for (int i = 0; i < n_sw_; i++)
	{
		//extract values for better readability
		double p_x = Xsig_aug(0, i);
		double p_y = Xsig_aug(1, i);
		double v = Xsig_aug(2, i);
		double yaw = Xsig_aug(3, i);
		double yawd = Xsig_aug(4, i);
		double nu_a = Xsig_aug(5, i);
		double nu_yawdd = Xsig_aug(6, i);
		//predicted state values
		double px_p, py_p;
		//avoid division by zero
		if (fabs(yawd) > 0.001) {
			px_p = p_x + v / yawd * (sin(yaw + yawd*delta_t) - sin(yaw));
			py_p = p_y + v / yawd * (cos(yaw) - cos(yaw + yawd*delta_t));
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
		Xsig_pred_(0, i) = px_p;
		Xsig_pred_(1, i) = py_p;
		Xsig_pred_(2, i) = v_p;
		Xsig_pred_(3, i) = yaw_p;
		Xsig_pred_(4, i) = yawd_p;
	}
	//cout << "Xsig_pred" << endl << Xsig_pred_ << endl; return;
	//Predicted Mean and Covariance---------------------------------------
	//predict state mean
	//x_.fill(0);
	for (int i = 0; i<n_sw_; i++)
		x_ += weights_(i) * Xsig_pred_.col(i);
	//predict state covariance matrix
	//P_.fill(0);
	VectorXd Xsig_pred_x = VectorXd(n_x_);
	Xsig_pred_x.fill(0.0);
	for (int i = 0; i<n_sw_; i++) {
		Xsig_pred_x = Xsig_pred_.col(i) - x_;
		//angle normalization
		/*int a7a = 0;
		while (Xsig_pred_x(3) > M_PI) {
			a7a++;
			if (a7a > 10) Xsig_pred_x(3) > M_PI;
			else Xsig_pred_x(3) -= 2.*M_PI; }
		cout << "1-a7a" << a7a << endl;
		a7a = 0;
		while (Xsig_pred_x(3) < -M_PI) {
			a7a++;
			if (a7a > 10) Xsig_pred_x(3) = -M_PI;
			else Xsig_pred_x(3) += 2.*M_PI; }
		cout << "2-a7a" << a7a << endl;*/
		Xsig_pred_x(3) = atan2(sin(Xsig_pred_x(3)), cos(Xsig_pred_x(3)));
		P_ += weights_(i)*Xsig_pred_x*Xsig_pred_x.transpose();
		
	}
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
}
