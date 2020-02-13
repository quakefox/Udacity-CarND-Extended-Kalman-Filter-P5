#include "kalman_filter.h"
#include <iostream>

using Eigen::MatrixXd;
using Eigen::VectorXd;


using std::cout;
using std::endl;


const float PI = 3.1415927;

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
  x_ = F_ * x_;
  MatrixXd Ft = F_.transpose();
  P_ = F_ * P_ * Ft + Q_;
  //cout << "KalmanFilter::Predict: " <<endl;
  //cout << "predicted x_= " << x_ << endl;
  //cout << "predicted P_= " << P_ << endl;
}

void KalmanFilter::Update(const VectorXd &z) {
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
  //cout << "KalmanFilter::Update: " <<endl;
  //cout << "predicted x_= " << x_ << endl;
  //cout << "predicted P_= " << P_ << endl;  


}

void KalmanFilter::UpdateEKF(const VectorXd &z) {
  VectorXd z_pred(3);
  float px = x_(0);
  float py = x_(1);
  float vx = x_(2);
  float vy = x_(3);

  if (fabs(px*px + py*py) < 0.0001) {
    cout << "UpdateEKF () - Error - Division by Zero" << endl;
    return;
  }

  float ro = sqrt(px*px + py*py);
  float theta = atan2(py,px);
  float ro_dot = (px*vx + py*vy)/ro; 
  z_pred << ro, theta, ro_dot;
  //cout << "z_pred= " << z_pred << endl;

  VectorXd y = z - z_pred;
  //normalize y angle
  if (y[1] < (-1)*PI) {
    y[1] += PI*2;
  } else if (y[1] > PI) {
    y[1] -= PI*2;
  } 

  MatrixXd Ht = H_.transpose();
  MatrixXd S = H_ * P_ * Ht + R_;
  MatrixXd Si = S.inverse();
  MatrixXd PHt = P_ * Ht;
  MatrixXd K = PHt * Si;

  x_ = x_ + (K * y);
  long x_size = x_.size();
  MatrixXd I = MatrixXd::Identity(x_size, x_size);
  P_ = (I - K * H_) * P_;

  //cout << "KalmanFilter::UpdateEKF: " <<endl;
  //cout << "predicted x_= " << x_ << endl;
  //cout << "predicted P_= " << P_ << endl;
}
