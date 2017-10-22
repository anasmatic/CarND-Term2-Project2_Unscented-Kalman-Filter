#include "ukf.h"
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
	use_laser_ = true;

	// if this is false, radar measurements will be ignored (except during init)
	use_radar_ = true;

	//[Anas : initialize dimintions vaules and lambda]
	is_initialized_ = false;
	n_x_ = 5;
	n_aug_ = n_x_ + 2;
	n_sig_ = 2 * n_aug_ + 1;
	lambda_ = 3 - n_x_;

	// initial state vector
	x_ = VectorXd(n_x_);

	// initial covariance matrix
	//P_ = MatrixXd(n_x_, n_x_);
	P_ = MatrixXd::Identity(n_x_, n_x_);//[Anas: MAN! I was happy to fin out about this Identity constructor]

	// Process noise standard deviation longitudinal acceleration in m/s^2
	std_a_ = .75;//1.5;//4;//7+15;//7;//15;//30;

	// Process noise standard deviation yaw acceleration in rad/s^2
	std_yawdd_ = .75;// .05;//7;//.75;//30;

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

	// weights
	weights_ = VectorXd(n_sig_);
	double weight_0 = lambda_ / (lambda_ + n_aug_);
	weights_(0) = weight_0;
	for (int i = 1; i<n_sig_; i++) {
		double weight = 0.5 / (n_aug_ + lambda_);
		weights_(i) = weight;
	}

	//[Anas: predicted sigma points matrix 
	Xsig_pred_ = MatrixXd(n_x_, n_sig_);

	/**
	TODO:
	Complete the initialization. See ukf.h for other member properties.
	Hint: one or more values initialized above might be wildly off...
	*/
}

UKF::~UKF() {}

/**
* @param {MeasurementPackage} meas_package The latest measurement data of
* either radar or laser.
*/
void UKF::ProcessMeasurement(MeasurementPackage meas_package) {
  /** TODO:  Complete this function! Make sure you switch between lidar and radar measurements.*/
	if (!is_initialized_) {
		if (meas_package.sensor_type_ == MeasurementPackage::RADAR) 
		{//[Anas:Convert radar Measurements from polar to cartesian coordinates ]
			double rho(meas_package.raw_measurements_[0]);
			double phi(meas_package.raw_measurements_[1]);
			double c_phi = cos(phi); double s_phi = sin(phi);
			double rho_dot(meas_package.raw_measurements_[2]);
			x_ << rho*c_phi, rho*s_phi, rho_dot, 0.0, 0.0;	
		}
		else if (meas_package.sensor_type_ == MeasurementPackage::LASER) 
		{
			double px(meas_package.raw_measurements_[0]);
			double py(meas_package.raw_measurements_[1]);
			x_ << px, py, 0.0, 0.0, 0.0;
		}
		time_us_ = meas_package.timestamp_;
		//[Anas:initialization done ]
		is_initialized_ = true;
		return;
	}

	//[Anas: get delta time in seconds]
	double dt((meas_package.timestamp_ - time_us_)*0.000001);

	if (dt>0) {//[Anas: avoid doublication]
		Prediction(dt);
		time_us_ = meas_package.timestamp_;
	}

	//[Anas: switch radar or lidar]
	if (meas_package.sensor_type_ == MeasurementPackage::LASER)
		UpdateLidar(meas_package);
	else
		UpdateRadar(meas_package);

	//cout << "x_" << endl << x_ << endl;
	//cout << "P_" << endl << P_ << endl;
}

/**
* Predicts sigma points, the state, and the state covariance matrix.
* @param {double} delta_t the change in time (in seconds) between the last
* measurement and this one.
*/
void UKF::Prediction(double delta_t) {
	/** TODO:
	Complete this function! Estimate the object's location. Modify the statevector, 
	x_. Predict sigma points, the state, and the state covariance matrix.
	*/
	MatrixXd Xsig_aug(n_aug_, n_sig_);
	AugmentSegmaPoints(Xsig_aug);//lesson 18
	PredictSigmaPoints(Xsig_aug, delta_t);//lesson 21

	//Predicted Mean and Covariance -- lesson 24
	VectorXd x = VectorXd(n_x_);
	MatrixXd P = MatrixXd(n_x_, n_x_);

	//predicted state mean
	x.fill(0.0);
	for (int i = 0; i < n_sig_; i++) {  //iterate over sigma points
		x = x + weights_(i) * Xsig_pred_.col(i);
	}

	//predicted state covariance matrix
	P.fill(0.0);
	for (int i = 0; i < n_sig_; i++) {  //iterate over sigma points

		// state difference
		VectorXd x_diff = Xsig_pred_.col(i) - x;
		//angle normalization
		x_diff(3) = atan2(sin(x_diff(3)), cos(x_diff(3)));
		
		P = P + weights_(i) * x_diff * x_diff.transpose();
	}

	//write result
	x_ = x;
	P_ = P;
	
}
void UKF::AugmentSegmaPoints(MatrixXd & Xsig_aug){
	
	//create augmented mean vector
	VectorXd x_aug = VectorXd(n_aug_);

	//create augmented state covariance
	MatrixXd P_aug = MatrixXd(n_aug_, n_aug_);

	//create augmented mean state
	x_aug.head(5) = x_;
	x_aug(5) = 0;
	x_aug(6) = 0;

	//create augmented covariance matrix
	P_aug.fill(0.0);
	P_aug.topLeftCorner(5, 5) = P_;
	P_aug(5, 5) = std_a_*std_a_;
	P_aug(6, 6) = std_yawdd_*std_yawdd_;

	//create square root matrix
	MatrixXd L = P_aug.llt().matrixL();

	//create augmented sigma points
	Xsig_aug.col(0) = x_aug;
	for (int i = 0; i< n_aug_; i++)
	{
		Xsig_aug.col(i + 1) = x_aug + sqrt(lambda_ + n_aug_) * L.col(i);
		Xsig_aug.col(i + 1 + n_aug_) = x_aug - sqrt(lambda_ + n_aug_) * L.col(i);
	}

//cout << "Xsig_aug" << Xsig_aug << endl;
}

void UKF::PredictSigmaPoints(MatrixXd Xsig_aug, double delta_t) {
	for (int i = 0; i< n_sig_; i++)
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
		if (fabs(yawd) > 1e-3) {
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
//	cout << "Xsig_pred_" << Xsig_pred_ << endl;
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
	//[Anas: I'll use EKF code, it is much less]
	int n_z = meas_package.raw_measurements_.size();// 2;
	
	MatrixXd H_ = MatrixXd(2, 5);
	H_ << 1, 0, 0, 0, 0,
					0, 1, 0, 0, 0;
	VectorXd z_pred = H_ * x_;
	VectorXd y = meas_package.raw_measurements_ - z_pred;//z_diff

	//SharedUpdate(y);
	MatrixXd R_ = MatrixXd(n_z, n_z);
	R_ << std_laspx_*std_laspx_	, 0				,
			0					, std_laspy_*std_laspy_;
	MatrixXd Ht = H_.transpose();
	MatrixXd S = H_ * P_ * Ht + R_;
	MatrixXd Si = S.inverse();
	MatrixXd PHt = P_ * Ht;
	MatrixXd K = PHt * Si;

	//new estimate
	x_ = x_ + (K * y);
	long x_size = x_.size();
	MatrixXd I = MatrixXd::Identity(x_size, x_size);
	P_ = (I - K * H_) * P_;

	NIS_laser = y.transpose() * Si * y;
}

/**
* Updates the state and the state covariance matrix using a radar measurement.
* @param {MeasurementPackage} meas_package
*/
void UKF::UpdateRadar(MeasurementPackage meas_package) {
	//lesson 27
	const int n_z = meas_package.raw_measurements_.size();// = 3
	MatrixXd Z_sig = MatrixXd(n_z, n_sig_);
	//transform sigma points into measurement space
	for (int i = 0; i < n_sig_; i++) {  
		// extract values for better readibility
		double p_x = Xsig_pred_(0, i);
		double p_y = Xsig_pred_(1, i);
		double v = Xsig_pred_(2, i);
		double yaw = Xsig_pred_(3, i);

		double v1 = cos(yaw)*v;
		double v2 = sin(yaw)*v;

		// measurement model
		Z_sig(0, i) = sqrt(p_x*p_x + p_y*p_y);                        //r
		Z_sig(1, i) = atan2(p_y, p_x);                                 //phi
		//TODO: avoid zero devision
		Z_sig(2, i) = (p_x*v1 + p_y*v2) / sqrt(p_x*p_x + p_y*p_y);   //r_dot
	}

	//mean predicted measurement
	VectorXd z_pred = VectorXd(n_z);
	z_pred.fill(0.0);
	for (int i = 0; i < n_sig_; i++) {
		z_pred = z_pred + weights_(i) * Z_sig.col(i);
	}

	//measurement covariance matrix S
	MatrixXd S = MatrixXd(n_z, n_z);
	S.fill(0.0);
	for (int i = 0; i < n_sig_; i++) {  //2n+1 simga points
											   //residual
		VectorXd z_diff = Z_sig.col(i) - z_pred;

		//angle normalization	//[Anas: replace while with atan2]
		z_diff(1) = atan2(sin(z_diff(1)), cos(z_diff(1)));
		
		S = S + weights_(i) * z_diff * z_diff.transpose();
	}

	//add measurement noise covariance matrix
	MatrixXd R = MatrixXd(n_z, n_z);
	R << std_radr_*std_radr_, 0, 0,
		0, std_radphi_*std_radphi_, 0,
		0, 0, std_radrd_*std_radrd_;
	S = S + R;

//	cout << "UKFUpdate" << meas_package.raw_measurements_.size() << endl;
	
	//lesson 30 UKF Update -----------------------------------

	//create matrix for cross correlation Tc
	MatrixXd Tc = MatrixXd(n_x_, n_z);
	//calculate cross correlation matrix
	Tc.fill(0.0);
	for (int i = 0; i < n_sig_; i++) {  //2n+1 simga points
												//residual
		VectorXd z_diff(Z_sig.col(i) - z_pred);
		//angle normalization	//[Anas: replace while with atan2]
		z_diff(1) = atan2(sin(z_diff(1)), cos(z_diff(1)));
		
		// state difference
		VectorXd x_diff = Xsig_pred_.col(i) - x_;
		//angle normalization	//[Anas: replace while with atan2]
		x_diff(3) = atan2(sin(x_diff(3)), cos(x_diff(3)));

		Tc = Tc + weights_(i) * x_diff * z_diff.transpose();
	}
	//Kalman gain K;
	MatrixXd Si = S.inverse();//[Anas:will be used later for NIS]
	MatrixXd K = Tc * Si;

	//residual
	VectorXd z_diff = meas_package.raw_measurements_ - z_pred;

	//angle normalization	//[Anas: replace while with atan2]
	z_diff(1) = atan2(sin(z_diff(1)), cos(z_diff(1)));
	
	//update state mean and covariance matrix 
	x_ = x_ + K * z_diff;
	P_ = P_ - K*S*K.transpose();

	NIS_radar = z_diff.transpose() * Si * z_diff;
}
