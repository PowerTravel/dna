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
		Chain();
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

		enum class Trace{
			RIGHT,		// Came From Positive X-dir
			LEFT,		// Came From Negative X-dir
			FORWARD,	// Came From Positive Y-dir
			BACKWARD,	// Came From Negative Y-dir
			UP,			// Came From Positive Z-dir
			DOWN,		// Came From Negative Z-dir
			START
		};


		int _N;
		std::vector<Sphere> _links;
		std::vector<Spring> _spring;

		const int _max = 1000;
		std::map<int, Trace> _globule;

		int hash_fun(int x, int y, int z);
		int hash_fun(Eigen::Vector3d x);
		Eigen::Vector3d get_idx(int i);
		Eigen::VectorXd get_prob_dist(Eigen::Vector3d pos);
};

#endif // CHAIN_HPP
