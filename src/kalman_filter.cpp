#include "kalman_filter.h"

using Eigen::MatrixXd;
using Eigen::VectorXd;

// Please note that the Eigen library does not initialize
// VectorXd or MatrixXd objects with zeros upon creation.

KalmanFilter::KalmanFilter() {}

KalmanFilter::~KalmanFilter() {}

void KalmanFilter::Predict() {
  x_ = F_ * x_;
  P_ = F_ * P_ * F_.transpose() + Q_;
}

void KalmanFilter::Update(const VectorXd &z) {
  // For laser measurements
  VectorXd z_pred = H_ * x_;
	VectorXd y = z - z_pred;       // Error term
	MatrixXd Ht = H_.transpose();
	MatrixXd S = H_ * P_ * Ht + R_;
	MatrixXd Si = S.inverse();
	MatrixXd PHt = P_ * Ht;
	MatrixXd K = PHt * Si;         // Kalman Gain

  //new estimate
	x_ = x_ + (K * y);
	long x_size = x_.size();
	MatrixXd I = MatrixXd::Identity(x_size, x_size);
	P_ = (I - K * H_) * P_;
}

void KalmanFilter::UpdateEKF(const VectorXd &z) {
  float rho = sqrt(x_[0]*x_[0]+x_[1]*x_[1]);
  float phi = atan2(x_[1],x_[0]);
  float rho_dot = (x_[0]*x_[2] + x_[1]*x_[3]) / rho;
  VectorXd h_function = VectorXd(3);
  h_function << rho, phi, rho_dot;
  VectorXd y = z - h_function;

  // Adjusting phi angle to keep it in given range [-pi,pi]
  phi = tools.Normalize_Phi(y[1]);
  y[1] = phi;

  MatrixXd Ht = H_.transpose();
  MatrixXd S = H_ * P_ * Ht + R_;
	MatrixXd Si = S.inverse();
	MatrixXd PHt = P_ * Ht;
	MatrixXd K = PHt * Si;         // Kalman Gain

  //new estimate
	x_ = x_ + (K * y);
	long x_size = x_.size();
	MatrixXd I = MatrixXd::Identity(x_size, x_size);
	P_ = (I - K * H_) * P_;
}
