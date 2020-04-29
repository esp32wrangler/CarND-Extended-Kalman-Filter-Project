#include "FusionEKF.h"
#include <iostream>
#include "Eigen/Dense"
#include "tools.h"

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
    
    ekf_.P_ = MatrixXd(4, 4);
    ekf_.P_ << 1, 0, 0, 0,
              0, 1, 0, 0,
              0, 0, 1000, 0,
              0, 0, 0, 1000;

    F_ = MatrixXd(4, 4);
    F_ << 1, 0, 1, 0,
              0, 1, 0, 1,
              0, 0, 1, 0,
              0, 0, 0, 1;
  // initializing matrices
  ekf_.R_laser_ = MatrixXd(2, 2);
  //measurement covariance matrix - laser
  ekf_.R_laser_ << 0.0225, 0,
                   0, 0.0225;

  //measurement covariance matrix - radar
  ekf_.R_radar_ = MatrixXd(3, 3);
  ekf_.R_radar_ << 0.09, 0, 0,
                   0, 0.0009, 0,
                   0, 0, 0.09;
    
  ekf_.H_laser_ = MatrixXd(2, 4);
  ekf_.H_laser_  << 1, 0, 0, 0,
            0, 1, 0, 0;

  // set the acceleration noise components
  noise_ax_ = 9;
  noise_ay_ = 9;

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

    ekf_.x_ = VectorXd(4);

    if (measurement_pack.sensor_type_ == MeasurementPackage::RADAR) {
      // TODO: Convert radar from polar to cartesian coordinates 
      //         and initialize state.
        float ro = measurement_pack.raw_measurements_(0);
        float fi = measurement_pack.raw_measurements_(1);
        float rodot = measurement_pack.raw_measurements_(2);
        float px = ro*cos(fi);
        float py = ro*sin(fi);
        float vx = rodot*cos(fi);
        float vy = rodot*sin(fi);
        
        ekf_.InitializeX( px, py, vx, vy);
        

    }
    else if (measurement_pack.sensor_type_ == MeasurementPackage::LASER) {
        
        std::cout << "Laser Initialization " << endl;

        // set the state with the initial location and zero velocity
        ekf_.InitializeX(measurement_pack.raw_measurements_[0], measurement_pack.raw_measurements_[1], 0, 0);

    }

    previous_timestamp_ = measurement_pack.timestamp_;

    // done initializing, no need to predict or update
    is_initialized_ = true;
    return;
  }

  /**
   * Prediction
   */

   
    float dt = (measurement_pack.timestamp_ - previous_timestamp_) / 1000000.0;
    previous_timestamp_ = measurement_pack.timestamp_;

    
    F_(0, 2) = dt;
    F_(1, 3) = dt;
    
    MatrixXd Q (4,4);
    
    float dt_2 = dt * dt;
    float dt_3 = dt_2 * dt;
    float dt_4 = dt_3 * dt;
    Q <<  dt_4/4*noise_ax_, 0, dt_3/2*noise_ax_, 0,
                0, dt_4/4*noise_ay_, 0, dt_3/2*noise_ay_,
                dt_3/2*noise_ax_, 0, dt_2*noise_ax_, 0,
                0, dt_3/2*noise_ay_, 0, dt_2*noise_ay_;
                
    ekf_.Predict(F_, Q);

    
  /**
   * Update
   */

  
    std::cout << "FE " << measurement_pack.raw_measurements_ <<  std::endl;

  if (measurement_pack.sensor_type_ == MeasurementPackage::RADAR) {
      MatrixXd Hj = Tools::CalculateJacobian(ekf_.x_);

      ekf_.UpdateEKF(measurement_pack.raw_measurements_, Hj);
  } else {
      ekf_.Update(measurement_pack.raw_measurements_);
  }

  // print the output
  cout << "x_ = " << ekf_.x_ << endl;
  cout << "P_ = " << ekf_.P_ << endl;
}
