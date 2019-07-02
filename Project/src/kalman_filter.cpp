#include "kalman_filter.h"
#include "float.h" /* FLT_EPSILON */
#include <math.h>       /* floor */

using Eigen::MatrixXd;
using Eigen::VectorXd;

/*
 * Please note that the Eigen library does not initialize
 *   VectorXd or MatrixXd objects with zeros upon creation.
 */

KalmanFilter::KalmanFilter()
{
    // state covariance matrix
    P_ = MatrixXd(4, 4);
    P_ << 1, 0, 0, 0,
        0, 1, 0, 0,
        0, 0, 1000, 0,
        0, 0, 0, 1000;

    // state transition matrix
    F_ = MatrixXd(4, 4);
    F_ << 1, 0, 0, 0,
        0, 1, 0, 0,
        0, 0, 1, 0,
        0, 0, 0, 1;

    // process covariance matrix
    Q_ = MatrixXd(4, 4);

    // state vector
    x_ = VectorXd(4);

}

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

/**
* predict the state
*/
void KalmanFilter::Predict()
{
    x_ = F_ * x_;
    MatrixXd Ft = F_.transpose();
    P_ = F_ * P_ * Ft + Q_;
}

/**
* update the state by using Kalman Filter equations
*/
void KalmanFilter::Update(const VectorXd &z)
{
    VectorXd z_pred = H_ * x_;
    VectorXd y = z - z_pred;
    MatrixXd Ht = H_.transpose();
    MatrixXd S = H_ * P_ * Ht + R_;
    MatrixXd Si = S.inverse();
    MatrixXd PHt = P_ * Ht;
    MatrixXd K = PHt * Si;

    //new estimate
    x_ = x_ + (K * y);
    long x_size = x_.size();
    MatrixXd I = MatrixXd::Identity(x_size, x_size);
    P_ = (I - K * H_) * P_;
}

float KalmanFilter::NormalizeAngle(const float& angle)
{
    float angle_norm = angle;

    // normalize angle between -pi to pi
    if (angle_norm > M_PI)
    {
        angle_norm -= M_PI * std::floor(angle_norm / M_PI);
    }

    if (angle_norm < -M_PI)
    {
        angle_norm += M_PI * std::floor(angle_norm / -M_PI);
    }

    return angle_norm;
}


/**
* update the state by using Extended Kalman Filter equations
*/
void KalmanFilter::UpdateEKF(const VectorXd &z)
{

    VectorXd y = z - H_EKF(x_);

    // normalize angle between -pi to pi
    y(1) = NormalizeAngle(y(1));

    MatrixXd Ht = H_.transpose();
    MatrixXd S = H_ * P_ * Ht + R_;

    MatrixXd K = P_ * Ht * S.inverse();

    // new estimate
    x_ = x_ + (K * y);
    long x_size = x_.size();
    MatrixXd I = MatrixXd::Identity(x_size, x_size);
    P_ = (I - K * H_) * P_;
}

/*
* The radar sensor outputs values in polar coordinates.
* The function h(x) maps values from Cartesian coordinates
* to polar coordinates.
*/
VectorXd KalmanFilter::H_EKF(const VectorXd &x)
{
    // The predicted measurement vector in polar coordinates
    VectorXd p(3);

    float px = x(0);
    float py = x(1);
    float vx = x(2);
    float vy = x(3);

    // convert to polar coordinates
    float rho = sqrt(px*px + py*py);
    float phi = atan2(py, px);

    // if rho is close to zero, then change it's value to minimal
    // in order to avoid division by 0
    if (rho < FLT_EPSILON)
    {
        rho = FLT_EPSILON;
    }

    float rho_dot = (px * vx + py * vy) / rho;

    p << rho, phi, rho_dot;

    return p;
}