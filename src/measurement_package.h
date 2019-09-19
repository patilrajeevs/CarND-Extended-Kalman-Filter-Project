#ifndef MEASUREMENT_PACKAGE_H_
#define MEASUREMENT_PACKAGE_H_

#include <iostream>
#include <sstream>
#include <vector>
#include "Eigen/Dense"

using std::string;
using std::istringstream;
using Eigen::MatrixXd;
using Eigen::VectorXd;

class MeasurementPackage {
 public:
  enum SensorType{
    LASER,
    RADAR
  } sensor_type_;

  Eigen::VectorXd raw_measurements_;
  long long timestamp_;
  Eigen::VectorXd gt_values;

  MeasurementPackage(string &line){
    istringstream iss(line);
    string sensor_type;
    iss >> sensor_type; // reads first element from the current line
    if (sensor_type.compare("L") == 0) {  // laser measurement
      // read measurements
      sensor_type_ = MeasurementPackage::LASER;
      raw_measurements_ = VectorXd(2);
      float px;
      float py;
      iss >> px;
      iss >> py;
      raw_measurements_ << px,py;
    } else if (sensor_type.compare("R") == 0) {
      // Skip Radar measurements
      sensor_type_ = MeasurementPackage::RADAR;
	  raw_measurements_ = VectorXd(3);
      float ro;
      float theta;
      float ro_dot;
      iss >> ro;
      iss >> theta;
      iss >> ro_dot;
      raw_measurements_ << ro,theta, ro_dot;
    }
    iss >> timestamp_;

    float x_gt;
    float y_gt;
    float vx_gt;
    float vy_gt;
    iss >> x_gt;
    iss >> y_gt;
    iss >> vx_gt;
    iss >> vy_gt;

    gt_values = VectorXd(4);
    gt_values << x_gt, y_gt, vx_gt, vy_gt;
    /* below is to be removed
    gt_values[0] = x_gt;
    gt_values[1] = y_gt;
    gt_values[2] = vx_gt;
    gt_values[3] = vy_gt;
    */
  }

  bool isRadar() const{
      return sensor_type_ == MeasurementPackage::RADAR;
  }

  bool isLidar() const{
      return sensor_type_ == MeasurementPackage::LASER;
  }

};

#endif // MEASUREMENT_PACKAGE_H_
