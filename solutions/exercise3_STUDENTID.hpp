#pragma once

/// do not change the name of the method
inline Eigen::MatrixXd getMassMatrix(const Eigen::VectorXd &gc)
{

  /// !!!!!!!!!! NO RAISIM FUNCTIONS HERE !!!!!!!!!!!!!!!!!
  // std::cout << gc.size() << std::endl;
  return Eigen::MatrixXd::Ones(gc.size() - 1, gc.size() - 1);
}