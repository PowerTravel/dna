#ifndef CHAIN_HPP
#define CHAIN_HPP

#include <vector>
#include <Eigen/Dense>
#include <map>

#include "Sphere.hpp"
#include "Spring.hpp"
class Chain
{

	public:
		// 
		Chain(std::map<int, int> m)
		Chain()
		virtual ~Chain();
		
		void update(double dt = 0.01);

		// Returns position of all the links
		Eigen::VectorXd getPos();
		// Returns Velocity of all the links
		Eigen::VectorXd getVel();

		// Returns potential and kinetic energy of the system
		Eigen::Vector2d getEnergy();

		void printChain();

		// Generera kedjan i en spatial haschmap
		void generateGlobule(int N);
	private:

		int _N;
		std::vector<Sphere> _links;
		std::vector<Spring> _spring;


		int spatial_hash_key_fun(int x, int y, int z, int max);
};

#endif // CHAIN_HPP
