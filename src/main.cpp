/*
* Created on 2026-04-01
* @Author: Saba Ghudushauri
*/

// ODE_solver.cpp : Defines the entry point for the application.
#include <vector>
#include <iostream>
#include <fstream>
#include "Eigen/Dense"
#include "ode/ode_all.hpp"
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


void static derivative(const Eigen::VectorXd& state, Eigen::VectorXd& d) {
	d[0] = state[1];
	// dv/dt = -k*x (assuming mass m = 1, k = 10.0)
	d[1] = -10.0 * state[0];
}
#include <chrono>

void static get_acc(const Eigen::VectorXd& pos, Eigen::VectorXd& acc) {
	// Spring sim
	double F = -10.0 * pos[0]; // Spring force = -k * x
	acc[0] = F; // Acceleration = Force (assuming mass = 1)
}
void static save_to_csv(const Eigen::MatrixXd& solution, const std::string& root_dir) {
	std::ofstream output_file(root_dir + "/simulation_results.csv");
	if (output_file.is_open()) {
		output_file << "Time,Position,Velocity\n";
		for (int i = 0; i < solution.rows(); ++i) {
			output_file << solution(i, 0) << "," << solution(i, 1) << "," << solution(i, 2) << "\n";
		}
		output_file.close();
		std::cout << "Results saved to simulation_results.csv. Steps taken: " << solution.rows() << std::endl;
	}
	else {
		std::cerr << "Unable to open file for writing." << std::endl;
	}
}
int main()
{
	std::string root_dir = PROJECT_ROOT_DIR;
	double y0 = 1.0; // Initial position (displacement from equilibrium)
	double v0 = 0.0; // Initial velocity
	double t_start = 0.0, t_end = 100.0;
	double dt = 0.01; // step size
	
	// Calculate total rows needed ahead of time
	int num_steps = static_cast<int>((t_end - t_start) / dt) + 1;

	Eigen::VectorXd state_initial(2); // Position, Velocity
	state_initial[0] = y0;
	state_initial[1] = v0;

	auto start_time = std::chrono::high_resolution_clock::now();
	// Calling RK45 function, passing the derivative function by name
	Eigen::MatrixXd solution = ode::rk45(derivative, state_initial, t_end, 1e-6, ode::dormand_prince);

	auto end_time = std::chrono::high_resolution_clock::now();
	std::chrono::duration<double, std::milli> duration = end_time - start_time;

	std::cout << "Simulation took: " << duration.count() << " ms"
		<< " (" << (duration.count() / 1000.0) << " seconds)\n";
	
	// // Save results to CSV file
	// Each row will contain: time, position, velocity
	save_to_csv(solution, root_dir);

	return 0;
}
