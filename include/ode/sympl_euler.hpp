#pragma once
#include "ode_includes.hpp"


namespace ode {
	// Symplectic Euler method implementation: 
	// Updating velocity first, then uses the new velocity to update position
	// We need to pass velocity separately to ensure that the position update uses the new velocity, not the old one
	// Way more accurate for simulating physics, for systems where energy conservation is important (orbital mechanics, spring systems, etc.)
	// The input function should only calculate acceleration based on the current position
	// Takes in separate initial position and velocity vectors
	// Returns a matrix where each row contains [positions, velocities]
	Eigen::MatrixXd sympl_euler(
		void(*get_acc)(const Eigen::VectorXd&, Eigen::VectorXd&),
		const Eigen::VectorXd& y0,
		const int& num_steps,
		const double& dt);
}