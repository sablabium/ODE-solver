#include "ode/rk4.hpp"

Eigen::MatrixXd rk4(
	void(*func)(const Eigen::VectorXd&, Eigen::VectorXd&),
	const Eigen::VectorXd& y0,
	const int& num_steps,
	const double& dt)
{
	const int dim = y0.size();
	Eigen::MatrixXd history(num_steps, y0.size());
	history.row(0) = y0;

	// Pre-allocating temporary vectors only once on the heap
	// Avoiding allocations inside function calls
	Eigen::VectorXd y(dim);
	Eigen::VectorXd k1(dim);
	Eigen::VectorXd k2(dim);
	Eigen::VectorXd k3(dim);
	Eigen::VectorXd k4(dim);
	Eigen::VectorXd temp(dim);

	for (int i = 1; i < num_steps; i++) {
		y = history.row(i - 1);

		// RK4 takes 4 mathematical samples of the derivative function
		// Func should be defined to take in the current state and modify the passed in vector with the derivative values
		// This optimizes for performance by avoiding multiple allocations and copies of the state vector INSIDE function calls
		func(y, k1);
		temp = y + (0.5 * dt) * k1;
		func(temp, k2);
		temp = y + (0.5 * dt) * k2;
		func(temp, k3);
		temp = y + dt * k3;
		func(temp, k4);

		// A weighted average updates the entire state vector simultaneously
		history.row(i) = y + (dt / 6.0) * (k1 + 2.0 * k2 + 2.0 * k3 + k4);
	}
	return history;
}