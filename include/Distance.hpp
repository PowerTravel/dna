#ifndef MEAN_DISTANCE_HPP
#define MEAN_DISTANCE_HPP

#include "Simulation.hpp"
#include "CollisionGrid.hpp"
class Distance: public Simulation
{
	public:
		Distance(std::map<std::string, std::string> sm);

		void apply();
		
	private:
		void init_simulaition_parameters(std::map<std::string, std::string> sm);
		void print(std::ostream& os);

		int _size;

};

#endif // MEAN_DISTANCE_HPP
