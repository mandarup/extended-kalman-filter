#include "kalman_filter.h"
#include <stdio.h>


using Eigen::MatrixXd;
using Eigen::VectorXd;

#include <iostream>
using namespace std;

const std::string red("\033[0;31m");
const std::string reset("\033[0m");

void NormalizeAngle(double& phi)
{
  phi = atan2(sin(phi), cos(phi));
}

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




  //float rho_dot = ( px * vx + py * vy )/ rho;
  // instead of relying on that px and py will not be zero at the same time,
  // do :
  const float eps = .0001;
  const float rho_dot = (px*vx + py*vy ) / std::max(eps, rho);

  VectorXd z_pred = VectorXd(3);
  z_pred << rho, theta, rho_dot;

  VectorXd y = z - z_pred;
  MatrixXd Ht = H_.transpose();
  MatrixXd PHt = P_ * Ht;
  MatrixXd S = H_ * PHt + R_;
  MatrixXd Si = S.inverse();
  MatrixXd K = PHt * Si;

  // normalize angle to (-pi,pi)
  //cout << red << "y: " << y  <<  ", PI: " << M_PI << reset << endl;


  for(int i=0; i < y.size(); i++){

    // This was updated to meet the required change from code review
    // But not clear how this will work if y[i] < -PI  to start with
    // Reviewer please explain
    // while (y[i] >  M_PI){
    //     y[i] -= 2 * M_PI;
    //     //cout << red << "y[" << i <<"] after normalizing: " << y[i] << reset  << endl;
    // }
    // while (y[i] <  -M_PI){
    //     y[i] += 2 * M_PI;
    //     //cout << red << "y[" << i <<"] after normalizing: " << y[i] << reset  << endl;
    // }

    NormalizeAngle(y[i]);
  }

  //new estimate
  x_ = x_ + (K * y);
  long x_size = x_.size();
  MatrixXd I = MatrixXd::Identity(x_size, x_size);
  P_ = (I - K * H_) * P_;
}
