#include "kalman_filter.h"
#include <stdio.h>


using Eigen::MatrixXd;
using Eigen::VectorXd;

#include <iostream>
using namespace std;

const std::string red("\033[0;31m");
const std::string reset("\033[0m");

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
  TODO:
    * predict the state
  */
  x_ = F_ * x_;
  MatrixXd Ft = F_.transpose();
  P_ = F_ * P_ * Ft + Q_;
}

void KalmanFilter::Update(const VectorXd &z) {
  /**
  TODO:
    * update the state by using Kalman Filter equations
  */

  VectorXd z_pred = H_ * x_;
	VectorXd y = z - z_pred;
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
}

void KalmanFilter::UpdateEKF(const VectorXd &z) {
  /**
  TODO:
    * update the state by using Extended Kalman Filter equations
  */
  float px = x_(0);
  float py = x_(1);
  float vx = x_(2);
  float vy = x_(3);

  float rho = sqrt(px *px + py *py );
  float theta = atan2(py,px);

  // cout << red << "Theta: " << theta  <<  ", PI: " << M_PI << reset << endl;
  // while (theta >  M_PI){
  //     theta -= 2 * M_PI;
  //     cout << red << "Theta after normalizing: " << theta <<  reset  << endl;
  // }
  // while (theta <  -M_PI){
  //     theta += 2 * M_PI;
  //     cout << red << "Theta after normalizing: " << theta << reset  << endl;
  // }

  float rho_dot = ( px * vx + py * vy )/ rho;

  VectorXd z_pred = VectorXd(3);
  z_pred << rho, theta, rho_dot;

  //VectorXd y = z - z_pred;
  //VectorXd z_pred = H_ * x_;

  VectorXd y = z - z_pred;
  MatrixXd Ht = H_.transpose();
  MatrixXd S = H_ * P_ * Ht + R_;
  MatrixXd Si = S.inverse();
  MatrixXd PHt = P_ * Ht;
  MatrixXd K = PHt * Si;

  cout << red << "y: " << y  <<  ", PI: " << M_PI << reset << endl;
  for(int i=0; i < y.size(); i++){
    while (y[i] >  M_PI){
        y[i] -= 2 * M_PI;
        cout << red << "y[" << i <<"] after normalizing: " << y[i] << reset  << endl;
    }
    while (y[i] <  -M_PI){
        y[i] += 2 * M_PI;
        cout << red << "y[" << i <<"] after normalizing: " << y[i] << reset  << endl;
    }
  }
  // cout << red << "y after normalizing: " << y <<  reset  << endl;



  //new estimate
  x_ = x_ + (K * y);
  long x_size = x_.size();
  MatrixXd I = MatrixXd::Identity(x_size, x_size);
  P_ = (I - K * H_) * P_;
}
