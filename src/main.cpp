// ODE_solver.cpp : Defines the entry point for the application.
#include <vector>
#include <iostream>
#include <fstream>
#include "Eigen/Dense"
#include "ode/std_euler.hpp"
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
// Symplectic Euler method implementation: 
// Updating velocity first, then uses the new velocity to update position
// We need to pass velocity separately to ensure that the position update uses the new velocity, not the old one
// Way more accurate for simulating physics, for systems where energy conservation is important (orbital mechanics, spring systems, etc.)
// The input function should only calculate acceleration based on the current position
// Takes in separate initial position and velocity vectors
// Returns a matrix where each row contains [positions, velocities]


void static derivative(const Eigen::VectorXd& state, Eigen::VectorXd& d) {
	const double pos = state[0];
	const double vel = state[1];

	const double F = -10.0 * pos;

	d[0] = vel;
	d[1] = F;
}
#include <chrono>

void static get_acc(const Eigen::VectorXd& pos, Eigen::VectorXd& acc) {
	// Spring sim
	double F = -10.0 * pos[0]; // Spring force = -k * x
	acc[0] = F; // Acceleration = Force (assuming mass = 1)
}
void static save_to_csv(const Eigen::MatrixXd& solution, double t_start, double dt) {
	std::ofstream output_file("../../../simulation_results.csv");
	if (output_file.is_open()) {
		output_file << "Time,Position,Velocity\n"; // CSV header
		for (int i = 0; i < solution.rows(); ++i) {
			double time = t_start + i * dt;
			output_file << time << "," << solution(i, 0) << "," << solution(i, 1) << "\n";
		}
		output_file.close();
		std::cout << "Results saved to simulation_results.csv" << std::endl;
	}
	else {
		std::cerr << "Unable to open file for writing." << std::endl;
	}
}
int main()
{
	double y0 = 5.0; // Initial position (displacement from equilibrium)
	double v0 = 0.0; // Initial velocity
	double t_start = 0.0, t_end = 10.0;
	double dt = 0.01; // step size
	
	// Calculate total rows needed ahead of time
	int num_steps = static_cast<int>((t_end - t_start) / dt) + 1;

	Eigen::VectorXd state_initial(2); // Position, Velocity
	state_initial[0] = y0;
	state_initial[1] = v0;

	auto start_time = std::chrono::high_resolution_clock::now();
	// Calling euler function, passing the derivative function by name
	Eigen::MatrixXd solution = std_euler(derivative, state_initial, num_steps, dt);
	//Eigen::MatrixXd solution = std_euler(get_acc, state_initial, num_steps, dt);

	auto end_time = std::chrono::high_resolution_clock::now();
	std::chrono::duration<double, std::milli> duration = end_time - start_time;

	std::cout << "Simulation took: " << duration.count() << " ms"
		<< " (" << (duration.count() / 1000.0) << " seconds)\n";
	
	// // Save results to CSV file
	// Each row will contain: time, position, velocity
	save_to_csv(solution, t_start, dt);

	return 0;
}
