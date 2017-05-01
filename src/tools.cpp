#include <iostream>
#include "tools.h"

using namespace std;
using Eigen::VectorXd;
using Eigen::MatrixXd;
using std::vector;

Tools::Tools() {}

Tools::~Tools() {}

VectorXd Tools::CalculateRMSE(const vector<VectorXd> &estimations,
                              const vector<VectorXd> &ground_truth) {
  VectorXd rmse(4);
  rmse << 0, 0, 0, 0;

  // Check validity of inputs
  if (estimations.size() != ground_truth.size() ||
      estimations.size() == 0) {
    cout << "Invalid estimation or ground_truth data" << endl;
    return rmse;
  }

  // Accumulate squared residuals
  for (int i = 0; i < estimations.size(); i++) {
    VectorXd residual = estimations[i] - ground_truth[i];
    residual = residual.array() * residual.array();
    rmse += residual;
  }

  // Calculate the mean
  rmse = rmse / estimations.size();

  // Calculate the squared root
  rmse = rmse.array().sqrt();

  return rmse;
}

MatrixXd Tools::CalculateJacobian(const VectorXd& x_state) {
  MatrixXd Hj(3, 4);
  const float px = x_state(0);
  const float py = x_state(1);
  const float vx = x_state(2);
  const float vy = x_state(3);

  if (px == 0 && py == 0) {
    cout << "CalculateJacobian () - Error - Divided by Zero" << endl;
    return Hj;
  }

  const float c1 = (px * px) + (py * py);
  const float c2 = sqrt(c1);
  const float c3 = c1 * c2;

  Hj << (px / c2), (py / c2), 0, 0,
        -(py / c1), (px / c1), 0, 0,
        (py * (vx*py - vy*px)) / c3, (px * (vx*py - vy*px)) / c3, (px / c2), (py / c2);

  return Hj;
}
