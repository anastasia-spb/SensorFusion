#include "tools.h"
#include <iostream>

using Eigen::VectorXd;
using Eigen::MatrixXd;
using std::vector;

Tools::Tools() {}

Tools::~Tools() {}

VectorXd Tools::CalculateRMSE(const vector<VectorXd> &estimations,
    const vector<VectorXd> &ground_truth)
{
    VectorXd rmse(4);
    rmse << 0, 0, 0, 0;

    if (estimations.empty())
    {
        std::cout << "Estimations vector is emty!" << std::endl;
        return rmse;
    }

    if (estimations.size() != ground_truth.size())
    {
        std::cout << "Estimations vector size and ground truth vector size are not equal!" << std::endl;
        return rmse;
    }

    // accumulate squared residuals
    for (auto i = 0; i < estimations.size(); ++i)
    {
        VectorXd residual = estimations[i] - ground_truth[i];

        // coefficient-wise multiplication
        residual = residual.array()*residual.array();
        rmse += residual;
    }

    rmse = rmse / estimations.size();

    // calculate the squared root
    rmse = rmse.array().sqrt();

    // return the result
    return rmse;
}

MatrixXd Tools::CalculateJacobian(const VectorXd& x_state)
{
    MatrixXd Hj(3, 4);
    // recover state parameters
    float px = x_state(0);
    float py = x_state(1);
    float vx = x_state(2);
    float vy = x_state(3);

    // check division by zero
    float sum = std::pow(px, 2) + std::pow(py, 2);
    // check division by zero
    if (fabs(sum) < 0.0001) {
        std::cout << "CalculateJacobian () - Error - Division by Zero" << std::endl;
        return Hj;
    }

    float sum_sq = std::sqrt(sum);
    float sum_sq_3 = std::pow(sum, 1.5f);



    // compute the Jacobian matrix
    float h11 = px / sum_sq;
    float h12 = py / sum_sq;

    float h21 = -py / sum;
    float h22 = px / sum;

    float h3_common = (vx*py - vy*px);
    float h31 = 0.0f;
    float h32 = 0.0f;
    if (h3_common != 0.0f)
    {
        h31 = py*h3_common / sum_sq_3;
        h32 = -px*h3_common / sum_sq_3;
    }

    Hj << h11, h12, 0.0f, 0.0f,
        h21, h22, 0.0f, 0.0f,
        h31, h32, h11, h12;

    return Hj;
}
