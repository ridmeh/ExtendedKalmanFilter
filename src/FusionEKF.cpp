#include "FusionEKF.h"
#include "tools.h"
#include "Eigen/Dense"
#include <iostream>

using namespace std;
using Eigen::MatrixXd;
using Eigen::VectorXd;
using std::vector;

/*
 * Constructor.
 */
FusionEKF::FusionEKF() {
  is_initialized_ = false;

  previous_timestamp_ =  1477010443049999;

  // initializing matrices
  R_laser_ = MatrixXd(2, 2);
  R_radar_ = MatrixXd(3, 3);
  H_laser_ = MatrixXd(2, 4);
  Hj_ = MatrixXd(3, 4);

  //measurement covariance matrix - laser
  R_laser_ << 0.0225, 0,
        0, 0.0225;

  //measurement covariance matrix - radar
  R_radar_ << 0.09, 0, 0,
        0, 0.0009, 0,
        0, 0, 0.09;

  /**
  TODO:
    * Finish initializing the FusionEKF.
    * Set the process and measurement noises
  */
  H_laser_ << 1, 0, 0, 0,
	  0, 1, 0, 0;

  Hj_ << 1, 1, 0, 0,
	  1, 1, 0, 0,
	  1, 1, 1, 1;

  //the initial transition matrix F_
  ekf_.F_ = MatrixXd(4, 4);
  ekf_.F_ << 1, 0, 1, 0,
	  0, 1, 0, 1,
	  0, 0, 1, 0,
	  0, 0, 0, 1;

  //state covariance matrix P
  ekf_.P_ = MatrixXd(4, 4);
  ekf_.P_ << 1, 0, 0, 0,
	  0, 1, 0, 0,
	  0, 0, 1000, 0,
	  0, 0, 0, 1000;

}

/**
* Destructor.
*/
FusionEKF::~FusionEKF() {}

void FusionEKF::ProcessMeasurement(const MeasurementPackage &measurement_pack) {
	/**
	 * Initialization
	 */
	if (!is_initialized_) {
		/**
		 * TODO: Initialize the state ekf_.x_ with the first measurement.
		 * TODO: Create the covariance matrix.
		 * You'll need to convert radar from polar to cartesian coordinates.
		 */

		 // first measurement
		cout << "EKF: " << endl;
		ekf_.x_ = VectorXd(4);
		ekf_.x_ << 1, 1, 1, 1;

		if (measurement_pack.sensor_type_ == MeasurementPackage::LASER) {
			// TODO: Convert radar from polar to cartesian coordinates 
			//         and initialize state.
			ekf_.x_(0) = measurement_pack.raw_measurements_(0);
			ekf_.x_(1) = measurement_pack.raw_measurements_(1);
		}
		else if (measurement_pack.sensor_type_ == MeasurementPackage::RADAR) {
			// TODO: Initialize state.
			double ro = measurement_pack.raw_measurements_(0);
			double theta = measurement_pack.raw_measurements_(1);
			double ro_dot = measurement_pack.raw_measurements_(2);

			// Refer : https://www.mathsisfun.com/polar-cartesian-coordinates.html

			// Conver r,0 to x,y co-ordindate
			ekf_.x_(0) = ro * cos(theta);
			ekf_.x_(1) = ro * sin(theta);
			ekf_.x_(2) = ro_dot * cos(theta);
			ekf_.x_(3) = ro_dot * sin(theta);

		}
		previous_timestamp_ = measurement_pack.timestamp_;

		// done initializing, no need to predict or update
		is_initialized_ = true;
		return;
	}

	/**
	 * Prediction
	 */

	 /**
	  * TODO: Update the state transition matrix F according to the new elapsed time.
	  * Time is measured in seconds.
	  * TODO: Update the process noise covariance matrix.
	  * Use noise_ax = 9 and noise_ay = 9 for your Q matrix.
	  */


	double dt = (measurement_pack.timestamp_ - previous_timestamp_) / 1000000.0;	//dt - expressed in seconds
	previous_timestamp_ = measurement_pack.timestamp_;


	double dt_2 = dt * dt;
	double dt_3 = dt_2 * dt;
	double dt_4 = dt_3 * dt;


	ekf_.F_(0, 2) = dt;
	ekf_.F_(1, 3) = dt;

	double noise_ax = 9;
	double noise_ay = 9;
	ekf_.Q_ = MatrixXd(4, 4);
	// Take time difference and calculate process co-variance matrix (Chp. 10)
	ekf_.Q_ << dt_4 / 4 * noise_ax, 0, dt_3 / 2 * noise_ax, 0,
		0, dt_4 / 4 * noise_ay, 0, dt_3 / 2 * noise_ay,
		dt_3 / 2 * noise_ax, 0, dt_2*noise_ax, 0,
		0, dt_3 / 2 * noise_ay, 0, dt_2*noise_ay;
	ekf_.Predict();

	/**
	 * Update
	 */

	 /**
	  * TODO:
	  * - Use the sensor type to perform the update step.
	  * - Update the state and covariance matrices.
	  */

	if (measurement_pack.sensor_type_ == MeasurementPackage::RADAR) {
		// TODO: Radar updates
		Tools tools;
		Hj_ = tools.CalculateJacobian(ekf_.x_);
		ekf_.H_ = Hj_;
		ekf_.R_ = R_radar_;
		ekf_.UpdateEKF(measurement_pack.raw_measurements_);

	}
	else if (measurement_pack.sensor_type_ == MeasurementPackage::LASER) {
		// TODO: Initialize state.
		ekf_.H_ = H_laser_;
		ekf_.R_ = R_laser_;
		ekf_.Update(measurement_pack.raw_measurements_);

	}

	// print the output
	cout << "x_ = " << ekf_.x_ << endl;
	cout << "P_ = " << ekf_.P_ << endl;
}
