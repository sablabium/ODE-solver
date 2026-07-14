#pragma once
#include "ode_includes.hpp"

namespace ode {
	enum step_method {
		fehlberg,
		dormand_prince,
		Cash_Karp,
	};

	Eigen::MatrixXd fehlberg_step();
	Eigen::MatrixXd dormand_prince_step();

	Eigen::MatrixXd rk45(
		void(*func)(const Eigen::VectorXd&, Eigen::VectorXd&),
		const Eigen::VectorXd& y0,
		const double& t_end,
		const double& tol = 1e-6,
		const step_method& method = dormand_prince
	);
}
