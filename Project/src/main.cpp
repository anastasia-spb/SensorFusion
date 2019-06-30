#include <math.h>
#include <sstream>
#include <iostream>
#include "json.hpp"
#include "FusionEKF.h"
#include "tools.h"

using Eigen::MatrixXd;
using Eigen::VectorXd;
using std::string;
using std::vector;


void fillMeasurementPackage(vector<MeasurementPackage>& measurement_pack_list, vector<VectorXd>& ground_truth)
{
    /**
    * Set Measurements
    */

    // hardcoded input file with laser and radar measurements
    string in_file_name_ = "obj_pose-laser-radar-synthetic-input.txt";
    std::ifstream in_file(in_file_name_.c_str(), std::ifstream::in);

    if (!in_file.is_open()) {
        std::cout << "Cannot open input file: " << in_file_name_ << std::endl;
    }

    string line;
    // set i to get only first 3 measurments
    int i = 0;
    while (getline(in_file, line) && (i <= 3)) {

        MeasurementPackage meas_package;

        std::istringstream iss(line);
        string sensor_type;
        iss >> sensor_type; // reads first element from the current line
        int64_t timestamp;
        if (sensor_type.compare("L") == 0) {  // laser measurement
                                              // read measurements
            meas_package.sensor_type_ = MeasurementPackage::LASER;
            meas_package.raw_measurements_ = VectorXd(2);
            float x;
            float y;
            iss >> x;
            iss >> y;
            meas_package.raw_measurements_ << x, y;
            iss >> timestamp;
            meas_package.timestamp_ = timestamp;
            measurement_pack_list.push_back(meas_package);

        }
        else if (sensor_type.compare("R") == 0) {
            meas_package.sensor_type_ = MeasurementPackage::RADAR;
            meas_package.raw_measurements_ = VectorXd(3);
            float ro;
            float theta;
            float ro_dot;
            iss >> ro;
            iss >> theta;
            iss >> ro_dot;
            meas_package.raw_measurements_ << ro, theta, ro_dot;
            iss >> timestamp;
            meas_package.timestamp_ = timestamp;
        }

        // read ground truth values
        float x_gt;
        float y_gt;
        float vx_gt;
        float vy_gt;
        iss >> x_gt;
        iss >> y_gt;
        iss >> vx_gt;
        iss >> vy_gt;

        VectorXd gt_values(4);
        gt_values(0) = x_gt;
        gt_values(1) = y_gt;
        gt_values(2) = vx_gt;
        gt_values(3) = vy_gt;
        ground_truth.push_back(gt_values);

        // increamed line counter
        ++i;
    }
}


void ProcessMeasurement(FusionEKF& fusionEKF, vector<VectorXd>& estimations, MeasurementPackage& meas_package)
{
    // Call ProcessMeasurement(meas_package) for Kalman filter
    fusionEKF.ProcessMeasurement(meas_package);

    // Push the current estimated x,y positon from the Kalman filter's
    //   state vector

    VectorXd estimate(4);

    double p_x = fusionEKF.ekf_.x_(0);
    double p_y = fusionEKF.ekf_.x_(1);
    double v1 = fusionEKF.ekf_.x_(2);
    double v2 = fusionEKF.ekf_.x_(3);

    estimate(0) = p_x;
    estimate(1) = p_y;
    estimate(2) = v1;
    estimate(3) = v2;

    estimations.push_back(estimate);
}

void calculateAndPrintRMSE(vector<VectorXd>& estimations, vector<VectorXd>& ground_truth)
{
    Tools tools;
    VectorXd RMSE = tools.CalculateRMSE(estimations, ground_truth);
    std::cout << RMSE << std::endl;
}


int main()
{

    // Create a Kalman Filter instance
    FusionEKF fusionEKF;

    // used to compute the RMSE later
    vector<VectorXd> estimations;
    vector<VectorXd> ground_truth;
    vector<MeasurementPackage> measurement_pack_list;

    fillMeasurementPackage(measurement_pack_list, ground_truth);
    for (auto& measurement : measurement_pack_list)
    {
        ProcessMeasurement(fusionEKF, estimations, measurement);
    }

    calculateAndPrintRMSE(estimations, ground_truth);

    return 0;
}