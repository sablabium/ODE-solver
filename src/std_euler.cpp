#include "ode/std_euler.hpp"

namespace ode {
	Eigen::MatrixXd std_euler(
		void(*func)(const Eigen::VectorXd&, Eigen::VectorXd&),
		const Eigen::VectorXd& y0,
		const int& num_steps,
		const double& dt)
	{
		const int dim = y0.size(); // Works for any dimensions automatically
		// Pre allocating the history matrix
		Eigen::MatrixXd history(num_steps, dim);

		Eigen::VectorXd current_state(dim);
		Eigen::VectorXd change(dim);

		// Inserting initial state to history
		history.row(0) = y0;

		for (int i = 1; i < num_steps; i++) {
			current_state = history.row(i - 1);
			// Calculating change
			// yn+1 = yn + f(x,y)*h
			func(current_state, change);
			history.row(i) = current_state + change * dt;
		}
		// Matrix containing the full simulation history
		return history;
	}
}