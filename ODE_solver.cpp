// ODE_solver.cpp : Defines the entry point for the application.
#include <vector>
#include <iostream>
#include <fstream>
#include "third_party/Eigen/Dense"

// y(n+1) = yn + h * f(tn, yn)
// h is the step size
// f is the derivative function

// ODE eulers method
// ydot(t) = f(t,y(t))
// y(t0) = y0
// f(t,y) is a given function
// t0 is a given initial time and y0 is a given initial value for y
// the unknown in the function is y(t)

// y0, h, f(x,y)
// yn+1 = yn + h * f(x,y) where xn+1 = xn + h

// Standard Euler method implementation:
// 
Eigen::MatrixXd std_euler(
	Eigen::VectorXd(*func)(const Eigen::VectorXd&),
	const Eigen::VectorXd& y0,
	int num_steps,
	double dt)
{
	// Pre allocating the history matrix
	Eigen::MatrixXd history(num_steps, y0.size());

	// Inserting initial state to history
	history.row(0) = y0;

	for (int i = 1; i < num_steps; i++) {
		Eigen::VectorXd prev_state = history.row(i - 1);

		// Calculating change
		// yn+1 = yn + f(x,y)*h
		Eigen::VectorXd change = func(prev_state) * dt;

		// New row = previous row + change
		history.row(i) = prev_state + change;
	}
	// Matrix containing the full simulation history
	return history;
}

// Symplectic Euler method implementation: 
// Updating velocity first, then uses the new velocity to update position
// We need to pass velocity separately to ensure that the position update uses the new velocity, not the old one
// Way more accurate for simulating physics, for systems where energy conservation is important (orbital mechanics, spring systems, etc.)
// The input function should only calculate acceleration based on the current position
// Takes in separate initial position and velocity vectors
// Returns a matrix where each row contains [positions, velocities]

Eigen::MatrixXd sympl_euler(
	Eigen::VectorXd(*get_acc)(const Eigen::VectorXd&),
	const Eigen::VectorXd& p0,
	const Eigen::VectorXd& v0,
	int num_steps,
	double dt)
{
	int dim = p0.size(); // Works for any dimensions automatically

	// Allocate space for the combined history [pos, vel]
	// first half of each row is position, second half is velocity
	Eigen::MatrixXd history(num_steps, dim * 2);

	// Set the initial states into the first row
	history.row(0).head(dim) = p0;
	history.row(0).tail(dim) = v0;

	for (int i = 1; i < num_steps; ++i) {
		Eigen::VectorXd prev_pos = history.row(i - 1).head(dim);
		Eigen::VectorXd prev_vel = history.row(i - 1).tail(dim);

		Eigen::VectorXd acc = get_acc(prev_pos);
		Eigen::VectorXd next_vel = prev_vel + acc * dt;
		Eigen::VectorXd next_pos = prev_pos + next_vel * dt;

		history.row(i).head(dim) = next_pos;
		history.row(i).tail(dim) = next_vel;
	}

	return history;
}

Eigen::MatrixXd rk4(
	Eigen::VectorXd(*func)(const Eigen::VectorXd&),
	const Eigen::VectorXd& y0,
	int num_steps,
	double dt)
{
	Eigen::MatrixXd history(num_steps, y0.size());
	history.row(0) = y0;

	for (int i = 1; i < num_steps; i++) {
		Eigen::VectorXd y = history.row(i - 1);

		// RK4 takes 4 blind mathematical samples of the derivative function
		Eigen::VectorXd k1 = func(y);
		Eigen::VectorXd k2 = func(y + 0.5 * dt * k1);
		Eigen::VectorXd k3 = func(y + 0.5 * dt * k2);
		Eigen::VectorXd k4 = func(y + dt * k3);

		// A weighted average updates the entire state vector simultaneously
		history.row(i) = y + (dt / 6.0) * (k1 + 2.0 * k2 + 2.0 * k3 + k4);
	}
	return history;
}

Eigen::VectorXd derivative(const Eigen::VectorXd& state) {
	Eigen::VectorXd d(state.size());
	// Spring sim
	// Fs = -k * |x| 
	// x is a displacement from equilibrium, k is the spring constant

	double F = -10.0 * state[0]; // Spring force = -k * x
	// index 0 is position, index 1 is velocity
	d[0] = state[1]; // Change in position = velocity
	d[1] = F; // Change in velocity = spring force

	return d;
}

int main()
{
	double y0 = 5.0; // Initial position (displacement from equilibrium)
	double v0 = 0.0; // Initial velocity
	double t_start = 0.0, t_end = 1000.0;
	double dt = 0.1; // step size
	
	// Calculate total rows needed ahead of time
	int num_steps = static_cast<int>((t_end - t_start) / dt) + 1;

	Eigen::VectorXd state_initial { {y0, v0} }; // Position, Velocity


	// Calling euler function, passing the derivative function by name
	Eigen::MatrixXd solution = std_euler(derivative, state_initial, num_steps, dt);

	// // Save results to CSV file
	// Each row will contain: time, position, velocity
	std::ofstream output_file("simulation_results.csv");
	if (output_file.is_open()) {
		output_file << "Time,Position,Velocity\n"; // CSV header
		for (int i = 0; i < solution.rows(); ++i) {
			double time = t_start + i * dt;
			output_file << time << "," << solution(i, 0) << "," << solution(i, 1) << "\n";
		}
		output_file.close();
		std::cout << "Results saved to simulation_results.csv" << std::endl;
	} else {
		std::cerr << "Unable to open file for writing." << std::endl;
	}

	return 0;
}
