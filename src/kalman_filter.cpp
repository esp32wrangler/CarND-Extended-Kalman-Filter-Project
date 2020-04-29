#include "kalman_filter.h"
#include "tools.h"
#include <iostream>

using Eigen::MatrixXd;
using Eigen::VectorXd;

/* 
 * Please note that the Eigen library does not initialize 
 *   VectorXd or MatrixXd objects with zeros upon creation.
 */

KalmanFilter::KalmanFilter() {
    x_ = VectorXd(4);
}

KalmanFilter::~KalmanFilter() {}


void KalmanFilter::InitializeX(float px, float py, float vx, float vy)
{
    x_ << px, py, vx, vy;
}

void KalmanFilter::Predict(MatrixXd& F, MatrixXd& Q) {
    x_ = F * x_;
    MatrixXd Ft = F.transpose();
    P_ = F * P_ * Ft + Q;
}

void KalmanFilter::Update(const VectorXd &z) {
    VectorXd z_pred = H_laser_ * x_;
    VectorXd y = z - z_pred;
    MatrixXd Ht = H_laser_.transpose();
    MatrixXd S = H_laser_ * P_ * Ht + R_laser_;
    MatrixXd Si = S.inverse();
    MatrixXd PHt = P_ * Ht;
    MatrixXd K = PHt * Si;

    //new estimate
    x_ = x_ + (K * y);
    long x_size = x_.size();
    MatrixXd I = MatrixXd::Identity(x_size, x_size);
    P_ = (I - K * H_laser_) * P_;
    //std::cout << x_(0) << " L , " << x_(1 )<< std::endl;

    
}


void KalmanFilter::UpdateEKF(const VectorXd &z, MatrixXd &Hj) {
    float ro = sqrt(x_(0)*x_(0) + x_(1) * x_(1));
    float fi = atan2(x_(1), x_(0));
    //std::cout << "fifi " << fi << " " << x_(1) << " " << x_(0) << std::endl;
    assert (fi > -M_PI && fi < M_PI);
    
    float rodot = (x_(0)*x_(2)+x_(1)*x_(3))/ro;
    VectorXd hx (3);
    hx << ro, fi, rodot;
    //std::cout << ro << " " << fi << " " << rodot << std::endl;
    
    VectorXd y (3);
    y = z - hx;
    
    while (y(1) > M_PI)
    {
        y(1) -= 2*M_PI;
    }
    while (y(1) < -M_PI)
    {
        y(1) += 2*M_PI;
    }
    
    MatrixXd S = Hj * P_ * Hj.transpose() + R_radar_;
    MatrixXd Si = S.inverse();
    MatrixXd K = P_ * Hj.transpose() * Si;
    
    long x_size = x_.size();
    MatrixXd I = MatrixXd::Identity(x_size, x_size);
    
    x_ = x_ + (K * y);
    //std::cout << x_(0) << "R , " << x_(1 )<< std::endl;

    P_ = (I - K * Hj) * P_;
    
}
