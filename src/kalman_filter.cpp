#include <iostream>
#include <cfloat>
#include "kalman_filter.h"

using namespace std;
using Eigen::MatrixXd;
using Eigen::VectorXd;

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
  const MatrixXd Ft = F_.transpose();
  x_ = F_ * x_;
  P_ = F_ * P_ * Ft + Q_;
}

void KalmanFilter::Update(const VectorXd &z) {
  /**
  TODO:
    * update the state by using Kalman Filter equations
  */
  const VectorXd y = z - H_ * x_;
  const MatrixXd Ht = H_.transpose();
  const MatrixXd S = H_ * P_ * Ht + R_;
  const MatrixXd Si = S.inverse();
  const MatrixXd I = MatrixXd::Identity(4, 4);
  const MatrixXd K =  P_ * Ht * Si;

  //new state
  x_ = x_ + (K * y);
  P_ = (I - (K * H_)) * P_;
}

void KalmanFilter::UpdateEKF(const VectorXd &z) {
  /**
  TODO:
    * update the state by using Extended Kalman Filter equations
  */

  VectorXd hx(3);
  const float px = x_(0);
  const float py = x_(1);
  const float vx = x_(2);
  const float vy = x_(3);

  hx(0) = sqrt(px*px + py*py);
  hx(1) = atan2(py, px);
  hx(2) = (px*vx + py*vy) / max(FLT_EPSILON, hx(0));

  VectorXd y = z - hx;

  // Normalize Phi in y vector so that it's angle is between -pi and pi
  while (y(1) < -M_PI) {
    y(1) += M_PI;
  }

  while (y(1) > M_PI) {
    y(1) -= M_PI;
  }

  const MatrixXd Ht = H_.transpose();
  const MatrixXd S = H_ * P_ * Ht + R_;
  const MatrixXd Si = S.inverse();
  const MatrixXd I = MatrixXd::Identity(4, 4);
  const MatrixXd K =  P_ * Ht * Si;

  //new state
  x_ = x_ + (K * y);
  P_ = (I - (K * H_)) * P_;
}
