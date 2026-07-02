#include "ode/sympl_euler.hpp"

namespace ode {
	Eigen::MatrixXd sympl_euler(
		void(*get_acc)(const Eigen::VectorXd&, Eigen::VectorXd&),
		const Eigen::VectorXd& y0,
		const int& num_steps,
		const double& dt)
	{
		int dim = y0.size() / 2; // Works for any dimensions automatically, assuming the input state vector is structured as [pos, vel]
		// Allocating space for the combined history [pos, vel]
		// first half of each row is position, second half is velocity
		Eigen::MatrixXd history(num_steps, dim * 2);

		Eigen::VectorXd pos = y0.head(dim);
		Eigen::VectorXd vel = y0.tail(dim);
		Eigen::VectorXd acc(dim);

		// Save initial state
		history.row(0).head(dim) = pos;
		history.row(0).tail(dim) = vel;

		for (int i = 1; i < num_steps; i++) {
			// Calculating acceleration using the local position vector (Fast cache hit)
			get_acc(pos, acc);

			// Updating local velocity and position (Blazing fast vector math)
			vel += acc * dt;
			// Using the updated velocity to update position (Symplectic step)
			// This is the key difference from standard Euler.
			// we use the new velocity to update position, not the old one
			pos += vel * dt;

			// Writing to history once per row
			history.row(i).head(dim) = pos;
			history.row(i).tail(dim) = vel;
		}

		return history;
	}
}