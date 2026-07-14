#include "ode/rk45.hpp"

namespace ode {
	Eigen::MatrixXd fehlberg_step();
	Eigen::MatrixXd dormand_prince_step();

	Eigen::MatrixXd rk45(
		void(*func)(const Eigen::VectorXd&, Eigen::VectorXd&),
		const Eigen::VectorXd& y0,
		const double& t_end,
		const double& tol = 1e-6,
		const step_method& method = dormand_prince
	)
	{
		double dt = 0.01;
		double t = 0;
		const int dim = y0.size();

		int current_capacity = 10'000;
		// Storing time in the first column and state in the subsequent columns
		Eigen::MatrixXd history(current_capacity, dim + 1);
		history(0, 0) = t;
		history.block(0, 1, 1, dim) = y0.transpose();

		int step_idx = 0;

		Eigen::VectorXd y = y0;
		Eigen::VectorXd k1(dim), k2(dim), k3(dim), k4(dim), k5(dim), k6(dim), k7(dim);
		Eigen::VectorXd temp(dim), error_vec(dim);

		while (t < t_end) {
			if (t + dt > t_end) {
				dt = t_end - t;
			}

			func(y, k1);
			temp = y + dt * (1.0 / 5.0) * k1;
			func(temp, k2);
			temp = y + dt * (3.0 / 40.0 * k1 + 9.0 / 40.0 * k2);
			func(temp, k3);
			temp = y + dt * (44.0 / 45.0 * k1 - 56.0 / 15.0 * k2 + 32.0 / 9.0 * k3);
			func(temp, k4);
			temp = y + dt * (19372.0 / 6561.0 * k1 - 25360.0 / 2187.0 * k2 + 64448.0 / 6561.0 * k3 - 212.0 / 729.0 * k4);
			func(temp, k5);
			temp = y + dt * (9017.0 / 3168.0 * k1 - 355.0 / 33.0 * k2 + 46732.0 / 5247.0 * k3 + 49.0 / 176.0 * k4 - 5103.0 / 18656.0 * k5);
			func(temp, k6);

			Eigen::VectorXd y_next = y + dt * (35.0 / 384.0 * k1 + 500.0 / 1113.0 * k3 + 125.0 / 192.0 * k4 - 2187.0 / 6784.0 * k5 + 11.0 / 84.0 * k6);

			func(y_next, k7);

			error_vec = dt * (71.0 / 57600.0 * k1 - 71.0 / 16695.0 * k3 + 71.0 / 1920.0 * k4 - 17253.0 / 339200.0 * k5 + 22.0 / 525.0 * k6 - 1.0 / 40.0 * k7);
			double error = error_vec.template lpNorm<Eigen::Infinity>();

			if (error == 0.0) error = 1e-16;
			double dt_new = dt * 0.9 * std::pow(tol / error, 0.2);

			if (error <= tol) {
				t += dt;
				y = y_next;
				step_idx++;

				if (step_idx >= current_capacity) {
					current_capacity *= 2;
					history.conservativeResize(current_capacity, Eigen::NoChange);
				}

				history(step_idx, 0) = t;
				history.block(step_idx, 1, 1, dim) = y.transpose();

				dt = std::min(dt_new, dt * 5.0);
			}
			else {
				dt = std::max(dt_new, dt * 0.1);
			}
		}

		history = history.topRows(step_idx + 1);
		return history;
	}
}