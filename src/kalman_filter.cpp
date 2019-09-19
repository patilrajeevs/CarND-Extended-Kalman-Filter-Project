#include "kalman_filter.h"
#include <cmath>

using Eigen::MatrixXd;
using Eigen::VectorXd;

/* 
 * Please note that the Eigen library does not initialize 
 *   VectorXd or MatrixXd objects with zeros upon creation.
 */

KalmanFilter::KalmanFilter() {}

KalmanFilter::~KalmanFilter() {}
/*
void KalmanFilter::Init(VectorXd &x_in, MatrixXd &P_in, MatrixXd &F_in,
                        MatrixXd &H_in, MatrixXd &R_in, MatrixXd &Q_in) {
  x_ = x_in;
  P_ = P_in;
  F_ = F_in;
  H_ = H_in;
  R_ = R_in;
  Q_ = Q_in;
}
*/
void KalmanFilter::Predict() {
  /**
   * TODO: predict the state
   */
    x_ = F_*x_;
    MatrixXd Ft = F_.transpose();
    P_ = F_*P_*Ft + Q_;

}

void KalmanFilter::Update(const VectorXd &z) {
  /**
   * TODO: update the state by using Kalman Filter equations
   */
  VectorXd z_pred = H_ * x_;
  VectorXd y = z - z_pred;
  MatrixXd Ht = H_.transpose();
  MatrixXd S = H_ * P_ * Ht + R_laser_;
  MatrixXd Si = S.inverse();
  MatrixXd PHt = P_ * Ht;
  MatrixXd K = PHt * Si;

  //new estimate
  x_ = x_ + (K * y);
  P_ = (I_ - K * H_) * P_;
}

void KalmanFilter::UpdateEKF(const VectorXd &z) {
  /**
   * TODO: update the state by using Extended Kalman Filter equations
   */
  float px = x_[0];
  float py = x_[1];
  float vx = x_[2];
  float vy = x_[3];


  // pre-compute a set of terms to avoid repeated calculation
  float c1 = sqrt(px*px+py*py);

  if(fabs(c1) < 0.0001){
    c1 = 0.0001;
  }

  VectorXd z_pred(3);
  z_pred[0] = c1;
  z_pred[1] = atan2(py, px);
  z_pred[2] = (px*vx + py*vy)/c1;

  VectorXd y = z - z_pred;
  //TODO
  y(1) = atan2(sin(y(1)), cos(y(1)));
  MatrixXd Hjt = Hj_.transpose();
  MatrixXd S = Hj_ * P_ * Hjt + R_radar_;
  MatrixXd Si = S.inverse();
  MatrixXd PHjt = P_ * Hjt;
  MatrixXd K =  PHjt * Si;

  x_ = x_ + (K * y);
  P_ = (I_ - K*Hj_) * P_;
}
