#ifndef CHAIN_HPP
#define CHAIN_HPP

#include <vector>
#include <Eigen/Dense>

#include "Sphere.hpp"
#include "Spring.hpp"
class Chain
{

	public:
		Chain(int N = 10 );
		virtual ~Chain();

		
		void update(double dt = 0.01);

		// Returns position of all the links
		Eigen::VectorXd getPos();
		// Returns Velocity of all the links
		Eigen::VectorXd getVel();

		// Returns potential and kinetic energy of the system
		Eigen::Vector2d getEnergy();


	private:
		int _N;
		std::vector<Sphere> _links;
		std::vector<Spring> _spring;
};

#endif // CHAIN_HPP
