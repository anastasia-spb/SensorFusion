#ifndef FusionEKF_H_
#define FusionEKF_H_

#include <fstream>
#include <string>
#include <vector>
#include "Eigen/Dense"
#include "kalman_filter.h"
#include "measurement_package.h"
#include "tools.h"

class FusionEKF
{
public:

    /**
    * Convert microseconds to seconds
    */
    static float MicrosecondToSeconds(const float& micro);

    /**
     * Constructor.
     */
    FusionEKF();

    /**
     * Destructor.
     */
    virtual ~FusionEKF() = default;

    /**
     * Run the whole flow of the Kalman Filter from here.
     */
    void ProcessMeasurement(const MeasurementPackage &measurement_pack);

    /**
     * Kalman Filter update and prediction math lives in here.
     */
    KalmanFilter ekf_;

private:

    /**
    * Process first measurement and change filter state to intialized
    */
    void ProcessFirstMeasurement(const MeasurementPackage &measurement_pack);

    // check whether the tracking toolbox was initialized or not (first measurement)
    bool is_initialized_;

    // previous timestamp
    MeasurementPackage::microsec previous_timestamp_;

    // tool object used to compute Jacobian and RMSE
    Tools tools;
    Eigen::MatrixXd R_laser_;
    Eigen::MatrixXd R_radar_;
    Eigen::MatrixXd H_laser_;
    Eigen::MatrixXd Hj_;

    float noise_ax_;
    float noise_ay_;
};

#endif // FusionEKF_H_
