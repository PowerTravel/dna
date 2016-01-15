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
		void init_simulaition_parameters(std::map<std::string, std::string> sm);
		void print(std::ostream& os);

		int _size;
		double _box_size;
		double _rad;

};

#endif // MEAN_DISTANCE_HPP
