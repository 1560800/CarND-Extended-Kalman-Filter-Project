#include "kalman_filter.h"
#include <iostream>

using Eigen::MatrixXd;
using Eigen::VectorXd;
using namespace std;

/* 
 * Please note that the Eigen library does not initialize 
 *   VectorXd or MatrixXd objects with zeros upon creation.
 */

KalmanFilter::KalmanFilter() {}

KalmanFilter::~KalmanFilter() {}

void KalmanFilter::Init(VectorXd &x_in, MatrixXd &P_in, MatrixXd &F_in,
                        MatrixXd &H_in, MatrixXd &R_in, MatrixXd &Q_in) {
  x_ = x_in;
  P_ = P_in;
  F_ = F_in;
  H_ = H_in;
  R_ = R_in;
  Q_ = Q_in;
}

void KalmanFilter::Predict() {
  /**
   * TODO: predict the state
   */
	x_ = F_ * x_;
	MatrixXd Ft = F_.transpose();
	P_ = F_ * P_ * Ft + Q_;
}

void KalmanFilter::Update(const VectorXd &z) {
  /**
   * TODO: update the state by using Kalman Filter equations
   */
	VectorXd z_pred = H_ * x_;
	VectorXd y = z - z_pred;
	Estimate(y, "LASER");
}

void KalmanFilter::UpdateEKF(const VectorXd &z) {
  /**
   * TODO: update the state by using Extended Kalman Filter equations
   */
	double px = x_(0);
	double py = x_(1);
	double px2 = px * px;
	double py2 = py * py;
	double vx = x_(2);
	double vy = x_(3);

	double rho = sqrt(px2 + py2);
	double theta = atan2(py, px);
	double rho_dot = 0;

	if ((px2 + py2) >= 0.0001) {
		rho_dot = (px * vx + py * vy) / rho;
	}
	else {
		cout << "UpdateEKF - Error - Division by Zero" << endl;
		rho_dot = 0;
	}

	VectorXd z_pred(3);
	z_pred << rho, theta, rho_dot;
	VectorXd y = z - z_pred;
	Estimate(y, "RADAR");
}

void KalmanFilter::Estimate(VectorXd &y, const string &sensor_type) {
	/**
	 * TODO: update the state by using Extended Kalman Filter equations
	 */
	MatrixXd Ht = H_.transpose();
	MatrixXd S = H_ * P_ * Ht + R_;
	MatrixXd Si = S.inverse();
	MatrixXd PHt = P_ * Ht;
	MatrixXd K = PHt * Si;
  
  if (sensor_type == "RADAR") {
    while (y(1) < -M_PI || M_PI < y(1)) {
      if (y(1) < -M_PI) {
        y(1) += 2 * M_PI;
      } else if (M_PI < y(1)) {
        y(1) -= 2 * M_PI;
      }
    }
  }
	//new estimate
	x_ = x_ + (K * y);
	long x_size = x_.size();
	MatrixXd I = MatrixXd::Identity(x_size, x_size);
	P_ = (I - K * H_) * P_;
}