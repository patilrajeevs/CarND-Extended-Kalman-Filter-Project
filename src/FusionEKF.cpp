#include "FusionEKF.h"
#include <iostream>
#include "Eigen/Dense"
#include "tools.h"
#include <cmath>

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
  ekf_.R_laser_ = MatrixXd(2, 2);
  ekf_.R_radar_ = MatrixXd(3, 3);
  ekf_.Hj_ = MatrixXd(3, 4);

  //measurement covariance matrix - laser
  ekf_.R_laser_ << 0.0225, 0,
              0, 0.0225;
  ekf_.F_ = MatrixXd(4,4);
  //measurement covariance matrix - radar
  ekf_.R_radar_ << 0.09, 0, 0,
              0, 0.0009, 0,
              0, 0, 0.09;

  /**
   * TODO: Finish initializing the FusionEKF.
   * TODO: Set the process and measurement noises
   */

  ekf_.x_ = VectorXd(4);
  ekf_.P_ = MatrixXd(4,4);
  ekf_.P_ << 1,0,0,0,
             0,1,0,0,
             0,0,1000,0,
             0,0,0,1000;
  ekf_.Q_ = MatrixXd(4,4);
  ekf_.H_ = MatrixXd(2,4);
  ekf_.H_ << 1,0,0,0,
             0,1,0,0;

  ekf_.I_ = MatrixXd::Identity(ekf_.x_.size(), ekf_.x_.size());
  noise_ax = 9;
  noise_ay = 9;
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
    ekf_.x_ << 1, 1, 1, 1;

    if (measurement_pack.isRadar()) {
      // TODO: Convert radar from polar to cartesian coordinates 
      //         and initialize state.
        float ro = measurement_pack.raw_measurements_[0];
        float theta = measurement_pack.raw_measurements_[1];
        ekf_.x_[0] = ro * cos(theta);
        ekf_.x_[0] = ro * sin(theta);
    }
    else if (measurement_pack.isLidar()) {
      // TODO: Initialize state.
        ekf_.x_[0] = measurement_pack.raw_measurements_[0];
        ekf_.x_[1] = measurement_pack.raw_measurements_[1];
        //ekf_.x_[2] = 0;
        //ekf_.x_[3] = 0;
    }
    else {
        cout << "Error, unknown measurement type" << endl;
        return;
    }
    // done initializing, no need to predict or update
    previous_timestamp_ = measurement_pack.timestamp_;
    is_initialized_ = true;
    return;
  }

  float dt = (measurement_pack.timestamp_ - previous_timestamp_)/1000000.0;
  previous_timestamp_ = measurement_pack.timestamp_;

  /**
   * Prediction
   */

  /**
   * TODO: Update the state transition matrix F according to the new elapsed time.
   * Time is measured in seconds.
   * TODO: Update the process noise covariance matrix.
   * Use noise_ax = 9 and noise_ay = 9 for your Q matrix.
   */
  ekf_.F_ << 1, 0, dt, 0,
            0, 1, 0, dt,
            0, 0, 1, 0,
            0, 0, 0, 1;
  // 2. Set the process covariance matrix Q
  ekf_.Q_ << (pow(dt,4)*noise_ax)/4, 0                     , (pow(dt,3)*noise_ax)/2, 0                     ,
            0                     , (pow(dt,4)*noise_ay)/4, 0                     , (pow(dt,3)*noise_ay)/2,
            (pow(dt,3)*noise_ax)/2, 0                     , pow(dt,2)*noise_ax    , 0                     ,
            0                     , (pow(dt,3)*noise_ay)/2, 0                     , pow(dt,2)*noise_ay    ;
  ekf_.Predict();
  /**
   * TODO:
   * - Use the sensor type to perform the update step.
   * - Update the state and covariance matrices.
   */

  if (measurement_pack.isRadar()) {
    // TODO: Radar updates
    ekf_.Hj_ = Tools().CalculateJacobian(ekf_.x_);
    ekf_.UpdateEKF(measurement_pack.raw_measurements_);
  } else if (measurement_pack.isLidar()){
    // TODO: Laser updates
    ekf_.Update(measurement_pack.raw_measurements_);
  }

  // print the output
  //cout << "x_ = " << ekf_.x_ << endl;
  //cout << "P_ = " << ekf_.P_ << endl;
}
