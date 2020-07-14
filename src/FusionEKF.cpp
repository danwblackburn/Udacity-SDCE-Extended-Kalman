#include "FusionEKF.h"
#include <iostream>
#include "Eigen/Dense"
#include "tools.h"
#include <math.h>

using Eigen::MatrixXd;
using Eigen::VectorXd;
using std::cout;
using std::endl;
using std::vector;

/**
 * Constructor.
 */
FusionEKF::FusionEKF()
{
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

  /**
   * Finish initializing the FusionEKF.
   * Set the process and measurement noises
   */
  //Laser measurement matrix
  H_laser_ << 1, 0, 0, 0,
              0, 1, 0, 0;

}

/**
 * Destructor.
 */
FusionEKF::~FusionEKF() {}

void FusionEKF::ProcessMeasurement(const MeasurementPackage &measurement_pack)
{
  /**
   * Initialization
   */
  if (!is_initialized_)
  {
    /**
     * TODO: Initialize the state ekf_.x_ with the first measurement.
     * TODO: Create the covariance matrix.
     * You'll need to convert radar from polar to cartesian coordinates.
     */

    // first measurement
    cout << "EKF: " << endl;
    ekf_.x_ = VectorXd(4);

    if (measurement_pack.sensor_type_ == MeasurementPackage::RADAR)
    {
      //convert from polar to cartesian
      double rho = measurement_pack.raw_measurements_[0];
      double phi = measurement_pack.raw_measurements_[1];
      double y = rho * sin(phi);
      double x = rho * cos(phi);

      //initialize state
      ekf_.x_ << x, y, 0, 0;
    }
    else if (measurement_pack.sensor_type_ == MeasurementPackage::LASER)
    {
      //initialize state vector
      ekf_.x_ <<  measurement_pack.raw_measurements_[0],
                  measurement_pack.raw_measurements_[1],
                  0,
                  0;        
    }

      //initialize covariance matrix
      ekf_.P_ = MatrixXd(4, 4);
      ekf_.P_ <<  1, 0, 0, 0,
                  0, 1, 0, 0,
                  0, 0, 1000, 0,
                  0, 0, 0, 1000;   

    //set previous timestamp to first measurment time
    previous_timestamp_ = measurement_pack.timestamp_;
    // done initializing, no need to predict or update
    is_initialized_ = true;
    return;
  }

  float micros_to_s = 1000000;
  float dt = (measurement_pack.timestamp_ - previous_timestamp_) / micros_to_s;
  previous_timestamp_ = measurement_pack.timestamp_;

  /**
   * Prediction
   */
  ekf_.Predict(dt);
  cout << "Predicted! " << endl;
  cout << ekf_.x_ << endl;
  /**
   * Update
   */
  if (measurement_pack.sensor_type_ == MeasurementPackage::RADAR)
  {
    ekf_.H_ = tools.CalculateJacobian(ekf_.x_);
    ekf_.R_ = R_radar_;
    VectorXd z = measurement_pack.raw_measurements_;
    cout << "Radar Preupdated! " << endl;
    ekf_.UpdateEKF(z);
    cout << "Radar Updated! " << endl;
  }
  else
  {
    ekf_.H_ = H_laser_;
    ekf_.R_ = R_laser_;
    VectorXd z = VectorXd(2);
    z <<  measurement_pack.raw_measurements_(0),
          measurement_pack.raw_measurements_(1),
    cout << "Laser Preupdated! " << endl;
    ekf_.Update(z);
    cout << "Laser Updated! " << endl;
  }

  // print the output
  cout << "x_ = " << ekf_.x_ << endl;
  cout << "P_ = " << ekf_.P_ << endl;
}
