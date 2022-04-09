#pragma once

/// do not change the name of the method
inline Eigen::MatrixXd getMassMatrix (const Eigen::VectorXd& gc) {

  /// !!!!!!!!!! NO RAISIM FUNCTIONS HERE !!!!!!!!!!!!!!!!!


  return Eigen::MatrixXd::Ones(18,18);
}

/// do not change the name of the method
inline Eigen::VectorXd getNonlinearities (const Eigen::VectorXd& gc, const Eigen::VectorXd& gv) {

  /// !!!!!!!!!! NO RAISIM FUNCTIONS HERE !!!!!!!!!!!!!!!!!


  return Eigen::VectorXd::Ones(18);
}

