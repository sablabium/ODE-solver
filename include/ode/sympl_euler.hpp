#pragma once
#include "ode_includes.hpp"

Eigen::MatrixXd sympl_euler(
	void(*get_acc)(const Eigen::VectorXd&, Eigen::VectorXd&),
	const Eigen::VectorXd& y0,
	const int& num_steps,
	const double& dt);