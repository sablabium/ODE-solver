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

void derivative(const Eigen::VectorXd& state, Eigen::VectorXd& d) {
	const double pos = state[0];
	const double vel = state[1];

	const double F = -10.0 * pos;

	d[0] = vel;
	d[1] = F;
}
#include <chrono>

void get_acc(const Eigen::VectorXd& pos, Eigen::VectorXd& acc) {
	// Spring sim
	double F = -10.0 * pos[0]; // Spring force = -k * x
	acc[0] = F; // Acceleration = Force (assuming mass = 1)
}
void save_to_csv(const Eigen::MatrixXd& solution, double t_start, double dt) {
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
	// Eigen::MatrixXd solution = std_euler(derivative, state_initial, num_steps, dt);
	Eigen::MatrixXd solution = sympl_euler(get_acc, state_initial, num_steps, dt);

	auto end_time = std::chrono::high_resolution_clock::now();
	std::chrono::duration<double, std::milli> duration = end_time - start_time;

	std::cout << "Simulation took: " << duration.count() << " ms"
		<< " (" << (duration.count() / 1000.0) << " seconds)\n";
	
	// // Save results to CSV file
	// Each row will contain: time, position, velocity
	save_to_csv(solution, t_start, dt);

	return 0;
}
