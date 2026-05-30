#pragma once
#include "ode_includes.hpp"

Eigen::MatrixXd rk4(
	void(*func)(const Eigen::VectorXd&, Eigen::VectorXd&),
	const Eigen::VectorXd& y0,
	const int& num_steps,
	const double& dt);