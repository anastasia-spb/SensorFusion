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

    // measurement matrix
    H_laser_ << 1.0, 0.0, 0.0, 0.0,
        0.0, 1.0, 0.0, 0.0;

    noise_ax_ = 9.0f;
    noise_ay_ = 9.0f;

}

float FusionEKF::MicrosecondToSeconds(const float& micro)
{
    return micro / 1000000.0f;
}

void FusionEKF::ProcessFirstMeasurement(const MeasurementPackage &measurement_pack)
{
    if (is_initialized_)
    {
        return;
    }

    // first measurement
    cout << "EKF: " << endl;

    previous_timestamp_ = measurement_pack.timestamp_;
    ekf_.x_ = VectorXd(4);

    if (measurement_pack.sensor_type_ == MeasurementPackage::RADAR) {
        // Convert radar from polar to cartesian coordinates
        // and initialize state.
        float rho_measured = measurement_pack.raw_measurements_[0];
        float phi_measured = measurement_pack.raw_measurements_[1];
        // convert from polar to cartesian coordinate system
        float px = rho_measured * cos(phi_measured);
        float py = rho_measured * sin(phi_measured);

        ekf_.x_ << px, py, 0, 0;

    }
    else if (measurement_pack.sensor_type_ == MeasurementPackage::LASER) {
        // Initialize state.

        float px = measurement_pack.raw_measurements_[0];
        float py = measurement_pack.raw_measurements_[1];

        ekf_.x_ << px, py, 0, 0;

    }
    else
    {
        std::cout << "Unknown sensor type." << std::endl;
        return;
    }

    is_initialized_ = true;
    return;
}

void FusionEKF::ProcessMeasurement(const MeasurementPackage &measurement_pack) {
    /**
     * Initialization
     */
    if (!is_initialized_)
    {
        ProcessFirstMeasurement(measurement_pack);
        return;
    }

    /**
     * Prediction
     */

     /**
      * Update the state transition matrix F according to the new elapsed time.
      * Time is measured in seconds.
      * Update the process noise covariance matrix.
      * Use noise_ax = 9 and noise_ay = 9 for your Q matrix.
      */

      //compute the time elapsed between the current and previous measurements
    MeasurementPackage::microsec dt_micro = measurement_pack.timestamp_ - previous_timestamp_;
    if (dt_micro <= 0)
    {
        throw("Negative timestep!");
    }

    // convert to seconds
    float dt = MicrosecondToSeconds(dt_micro);

    float dt_2 = dt   * dt;
    float dt_3 = dt_2 * dt;
    float dt_4 = dt_3 * dt;

    // modify the state transition matrix F so that the time is integrated
    ekf_.F_(0, 2) = dt;
    ekf_.F_(1, 3) = dt;

    // update the process covariance matrix
    ekf_.Q_ << dt_4 / 4 * noise_ax_, 0.0, dt_3 / 2 * noise_ax_, 0.0,
        0.0, dt_4 / 4 * noise_ay_, 0.0, dt_3 / 2 * noise_ay_,
        dt_3 / 2 * noise_ax_, 0.0, dt_2*noise_ax_, 0.0,
        0.0, dt_3 / 2 * noise_ay_, 0.0, dt_2*noise_ay_;


    // update timestamp
    previous_timestamp_ = measurement_pack.timestamp_;

    ekf_.Predict();

    /**
     * Update
     */

     /**
      * Use the sensor type to perform the update step.
      * Update the state and covariance matrices.
      */

    if (measurement_pack.sensor_type_ == MeasurementPackage::RADAR)
    {
        // Radar updates
        Hj_ = tools.CalculateJacobian(ekf_.x_);
        ekf_.H_ = Hj_;
        ekf_.R_ = R_radar_;
        ekf_.UpdateEKF(measurement_pack.raw_measurements_);

    }
    else {
        // Laser updates
        ekf_.H_ = H_laser_;
        ekf_.R_ = R_laser_;
        ekf_.Update(measurement_pack.raw_measurements_);

    }

    // print the output
    cout << "x_ = " << ekf_.x_ << endl;
    cout << "P_ = " << ekf_.P_ << endl;
}
