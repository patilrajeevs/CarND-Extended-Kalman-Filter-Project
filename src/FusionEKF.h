#ifndef FusionEKF_H_
#define FusionEKF_H_

#include <fstream>
#include <string>
#include <vector>
#include "Eigen/Dense"
#include "kalman_filter.h"
#include "measurement_package.h"
#include "tools.h"

class FusionEKF {
 public:
  /**
   * Constructor.
   */
  FusionEKF();

  /**
   * Destructor.
   */
  virtual ~FusionEKF();

  /**
   * Run the whole flow of the Kalman Filter from here.
   */
  void ProcessMeasurement(const MeasurementPackage &measurement_pack);

  /**
   * Kalman Filter update and prediction math lives in here.
   */
  KalmanFilter ekf_;

 private:
  // check whether the tracking toolbox was initialized or not (first measurement)
  bool is_initialized_;

  // previous timestamp
  long long previous_timestamp_;

  // tool object used to compute Jacobian and RMSE
  Tools tools;
  float noise_ax;
  float noise_ay;
};

#endif // FusionEKF_H_
