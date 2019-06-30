#include <iostream>
#include <vector>
#include "Dense"

using Eigen::MatrixXd;
using Eigen::VectorXd;
using std::cout;
using std::endl;


MatrixXd CalculateJacobian(const VectorXd& x_state) {

    MatrixXd Hj(3, 4);
    // recover state parameters
    float px = x_state(0);
    float py = x_state(1);
    float vx = x_state(2);
    float vy = x_state(3);

    // TODO: YOUR CODE HERE

    // check division by zero
    float sum = std::pow(px, 2) + std::pow(py, 2);
    // check division by zero
    if (fabs(sum) < 0.0001) {
        cout << "CalculateJacobian () - Error - Division by Zero" << endl;
        return Hj;
    }

    float sum_sq = std::sqrt(sum);
    float sum_sq_3 = std::pow(sum, 1.5);



    // compute the Jacobian matrix
    float h11 = px / sum_sq;
    float h12 = py / sum_sq;

    float h21 = -py / sum;
    float h22 = px / sum;

    float h3_common = (vx*py - vy*px);
    float h31 = 0.0;
    float h32 = 0.0;
    if (h3_common != 0)
    {
        h31 = py*h3_common / sum_sq_3;
        h32 = -px*h3_common / sum_sq_3;
    }

    Hj << h11, h12, 0.0, 0.0,
        h21, h22, 0.0, 0.0,
        h31, h32, h11, h12;

    return Hj;
}

int test_jacobian() {
    /**
    * Compute the Jacobian Matrix
    */

    // predicted state example
    // px = 1, py = 2, vx = 0.2, vy = 0.4
    VectorXd x_predicted(4);
    x_predicted << 1, 2, 0.2, 0.4;

    MatrixXd Hj = CalculateJacobian(x_predicted);

    cout << "Hj:" << endl << Hj << endl;

    return 0;
}