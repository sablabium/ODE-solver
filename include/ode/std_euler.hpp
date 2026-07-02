#pragma once
#include "ode_includes.hpp"

namespace ode {
	// Standard Euler method implementation:
	// 
	Eigen::MatrixXd std_euler(
		void(*func)(const Eigen::VectorXd&, Eigen::VectorXd&),
		const Eigen::VectorXd& y0,
		const int& num_steps,
		const double& dt);
}