#include "kalman_filter.h"
#include <iostream>
#include <math.h>

using std::cout;
using std::endl;
using Eigen::MatrixXd;
using Eigen::VectorXd;

/* 
 * Please note that the Eigen library does not initialize 
 *   VectorXd or MatrixXd objects with zeros upon creation.
 */

KalmanFilter::KalmanFilter() {}

KalmanFilter::~KalmanFilter() {}

void KalmanFilter::Init(VectorXd &x_in, MatrixXd &P_in, MatrixXd &F_in,
                        MatrixXd &H_in, MatrixXd &R_in, MatrixXd &Q_in) 
{
  x_ = x_in;
  P_ = P_in;
  F_ = F_in;
  H_ = H_in;
  R_ = R_in;
  Q_ = Q_in;
}

void KalmanFilter::Predict(float dt) 
{
  //state transition matrix
  F_ << 1, 0, dt, 0,
        0, 1, 0, dt,
        0, 0, 1, 0,
        0, 0, 0, 1;

  //variables to calculate process covariance
  float noise_ax = 9;
  float noise_ay = 9;
  float dt_2 = dt * dt;
  float dt_3 = dt_2 * dt;
  float dt_4 = dt_3 * dt;

  //Calculate process covariance matrix
  Q_ <<   dt_4/4*noise_ax, 0, dt_3/2*noise_ax, 0,
          0, dt_4/4*noise_ay, 0, dt_3/2*noise_ay,
          dt_3/2*noise_ax, 0, dt_2*noise_ax, 0,
          0, dt_3/2*noise_ay, 0, dt_2*noise_ay;
  
  //Predict new state and covariance
  x_ = F_ * x_;
  MatrixXd Ft = F_.transpose();
  P_ = F_ * P_ * Ft + Q_;
}

void KalmanFilter::Update(const VectorXd &z) 
{
  VectorXd z_pred = H_ * x_;
  VectorXd y = z - z_pred;
  MatrixXd Ht = H_.transpose();
  MatrixXd S = (H_ * P_ * Ht )+ R_;
  MatrixXd Si = S.inverse();
  MatrixXd PHt = P_ * Ht;
  MatrixXd K = PHt * Si;

  //new estimate
  x_ = x_ + (K * y);
  int x_size = x_.size();
  MatrixXd I = MatrixXd::Identity(x_size, x_size);
  P_ = (I - K * H_) * P_;
}

void KalmanFilter::UpdateEKF(const VectorXd &z) 
{
  //Dissect state matrix
  float px = x_(0);
  float py = x_(1);
  float vx = x_(2);
  float vy = x_(3);

  //H function to convert state vector from cartesian to polar
  VectorXd h_func = VectorXd(3);
  h_func << sqrt(pow(px, 2) + pow(py,2)),
            atan2(py, px),
            (px*vx + py*vy) / sqrt((px*px) + (py*py));

  //Calculate error and make sure it is in bounds
  VectorXd y = z - h_func;
  while(y[1] > M_PI)
  {
    y[1] = y[1] - 2 * M_PI;
  }
  while(y[1] < - M_PI)
  {
    y[1] = y[1] + 2 * M_PI;
  }

  //Calculate kalman filter gain matrix
  MatrixXd Ht = H_.transpose();
  MatrixXd S = (H_ * P_ * Ht )+ R_;
  MatrixXd Si = S.inverse();
  MatrixXd PHt = P_ * Ht;
  MatrixXd K = PHt * Si;

  //new estimate
  x_ = x_ + (K * y);
  int x_size = x_.size();
  MatrixXd I = MatrixXd::Identity(x_size, x_size);
  P_ = (I - K * H_) * P_;
}
