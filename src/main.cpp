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
	Eigen::MatrixXd solution = rk45(derivative, state_initial, t_end, 1e-6, dormand_prince);

	auto end_time = std::chrono::high_resolution_clock::now();
	std::chrono::duration<double, std::milli> duration = end_time - start_time;

	std::cout << "Simulation took: " << duration.count() << " ms"
		<< " (" << (duration.count() / 1000.0) << " seconds)\n";
	
	// // Save results to CSV file
	// Each row will contain: time, position, velocity
	save_to_csv(solution, root_dir);

	return 0;
}
