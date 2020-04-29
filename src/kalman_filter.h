#ifndef KALMAN_FILTER_H_
#define KALMAN_FILTER_H_

#include "Eigen/Dense"

class KalmanFilter {
 public:
  /**
   * Constructor
   */
  KalmanFilter();

  /**
   * Destructor
   */
  virtual ~KalmanFilter();

  void InitializeX(float px, float py, float vx, float vy);
  /**
   * Prediction Predicts the state and the state covariance
   * using the process model
   */
  void Predict(Eigen::MatrixXd& F, Eigen::MatrixXd& Q);

  /**
   * Updates the state by using standard Kalman Filter equations
   * @param z The measurement at k+1
   */
  void Update(const Eigen::VectorXd &z);

  /**
   * Updates the state by using Extended Kalman Filter equations
   * @param z The measurement at k+1
   */
  void UpdateEKF(const Eigen::VectorXd &z, Eigen::MatrixXd &Hj);

  // state vector
  Eigen::VectorXd x_;

  // state covariance matrix
  Eigen::MatrixXd P_;


  // measurement matrix
  Eigen::MatrixXd H_laser_;

  // measurement covariance matrix
  Eigen::MatrixXd R_laser_;
  
    // measurement covariance matrix
  Eigen::MatrixXd R_radar_;
};

#endif // KALMAN_FILTER_H_
