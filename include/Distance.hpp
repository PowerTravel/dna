#ifndef MEAN_DISTANCE_HPP
#define MEAN_DISTANCE_HPP

#include "Simulation.hpp"
#include "CollisionGrid.hpp"
#include "Particle.hpp"
class Distance: public Simulation
{
	public:
		Distance(std::map<std::string, std::string> sm);

		void apply();

	private:

		Eigen::ArrayXXd run_simulation_once(double radius, 
							Eigen::Vector3d x_ini, Eigen::Vector3d v_ini, 
							double tot_time, double dt, 
							CollisionGrid* cg);

		void init_simulaition_parameters(std::map<std::string, std::string> sm);
		void print(std::ostream& os);



		Eigen::ArrayXXd P;		// Position
		Eigen::ArrayXXd P_var;	// Position variance
		Eigen::ArrayXd D;		// Distance
		Eigen::ArrayXd D_var;	// Distance Variance

		int nr_simulations;
		int nr_data_points;
		Eigen::ArrayXd time_steps;
		Eigen::ArrayXd time_step_mean;

		int _size;
		double _box_size;
		double _rad;

		int get_max(Eigen::Array3d v);

		void write_to_terminal(int i, int j);
		void write_to_file();
};

#endif // MEAN_DISTANCE_HPP
