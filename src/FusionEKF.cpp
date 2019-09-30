#include "FusionEKF.h"
#include <iostream>
#include "Eigen/Dense"
#include "tools.h"

using Eigen::MatrixXd;
using Eigen::VectorXd;
using std::cout;
using std::endl;
using std::vector;

/**
 * Constructor.
 */

FusionEKF::FusionEKF() {
  is_initialized_ = false;

  previous_timestamp_ = 0;

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

  //measurement function matrix H
  H_laser_ << 1, 0, 0, 0,
		0, 1, 0, 0;

  //jacobian matrix H
  Hj_ << 0, 0, 0, 0,
	0, 0, 0, 0,
	0, 0, 0, 0;
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

    if (measurement_pack.sensor_type_ == MeasurementPackage::RADAR) {
      // TODO: Convert radar from polar to cartesian coordinates 
      //         and initialize state.
	  // set the state with the initial location and zero velocity
		float ro = measurement_pack.raw_measurements_[0];
		float theta = measurement_pack.raw_measurements_[1];
		float ro_dot = measurement_pack.raw_measurements_[2];

		double px = ro * cos(theta);
		double py = ro * sin(theta);
		double vx = ro_dot * cos(theta);
		double vy = ro_dot * sin(theta);

		ekf_.x_ << px, py, vx, vy;
    }
    else if (measurement_pack.sensor_type_ == MeasurementPackage::LASER) {
      // TODO: Initialize state.
		ekf_.x_ << measurement_pack.raw_measurements_[0],
			measurement_pack.raw_measurements_[1],
			0,
			0;
    }

	previous_timestamp_ = measurement_pack.timestamp_;
    // done initializing, no need to predict or update
    is_initialized_ = true;
    return;
  }

  /**----------------------------------------------------------------------------------
   * Prediction
   -----------------------------------------------------------------------------------*/
  /**
   * TODO: Update the state transition matrix F according to the new elapsed time.
   * Time is measured in seconds.
   * TODO: Update the process noise covariance matrix.
   * Use noise_ax = 9 and noise_ay = 9 for your Q matrix.
   */

 // compute the time elapsed between the current and previous measurements
 // dt - expressed in seconds
  float dt = (measurement_pack.timestamp_ - previous_timestamp_) / 1000000.0;
  previous_timestamp_ = measurement_pack.timestamp_;

  // TODO: YOUR CODE HERE
  float dt_2 = dt * dt;
  float dt_3 = dt_2 * dt;
  float dt_4 = dt_3 * dt;

  // Modify the F matrix so that the time is integrated
  ekf_.F_ = MatrixXd(4, 4);
  ekf_.F_ <<  1, 0, dt, 0,
			  0, 1, 0, dt,
			  0, 0, 1, 0,
			  0, 0, 0, 1;
  // set the acceleration noise components
  float noise_ax = 5;
  float noise_ay = 5;

  // set the process covariance matrix Q
  ekf_.Q_ = MatrixXd(4, 4);
  ekf_.Q_ << dt_4 / 4 * noise_ax, 0, dt_3 / 2 * noise_ax, 0,
	  0, dt_4 / 4 * noise_ay, 0, dt_3 / 2 * noise_ay,
	  dt_3 / 2 * noise_ax, 0, dt_2* noise_ax, 0,
	  0, dt_3 / 2 * noise_ay, 0, dt_2* noise_ay;

  ekf_.Predict();

  /**----------------------------------------------------------------------------------
   * Update
   -----------------------------------------------------------------------------------*/

  /**
   * TODO:
   * - Use the sensor type to perform the update step.
   * - Update the state and covariance matrices.
   */

  if (measurement_pack.sensor_type_ == MeasurementPackage::RADAR) {
    // TODO: Radar updates
	  float px = ekf_.x_[0];
	  float py = ekf_.x_[1];

	  //check error on division by zero
	  if ((px * px + py * py) >= 0.0001) {
		  Hj_ = tools.CalculateJacobian(ekf_.x_);
		  ekf_.H_ = Hj_;
		  ekf_.R_ = R_radar_;
		  ekf_.UpdateEKF(measurement_pack.raw_measurements_);
	  }
	  else {
		  cout << "CalculateJacobian () - Error - Division by Zero" << endl;
	  }

  } else {
    // TODO: Laser updates
	  ekf_.H_ = H_laser_;
	  ekf_.R_ = R_laser_;
	  ekf_.UpdateEKF(measurement_pack.raw_measurements_);
  }

  // print the output
  cout << "x_ = " << ekf_.x_ << endl;
  cout << "P_ = " << ekf_.P_ << endl;
}
