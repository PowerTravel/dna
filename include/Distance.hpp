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

		int _size;
		double _box_size;
		double _rad;

		int get_max(Eigen::Array3d v);
};

#endif // MEAN_DISTANCE_HPP
