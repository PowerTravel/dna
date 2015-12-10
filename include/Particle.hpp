#ifndef PARTICLE_HPP
#define PARTICLE_HPP

#include "CollisionGrid.hpp"
#include "Sphere.hpp"
#include "Plane.hpp"
class Particle{

	public:
		Particle(double rad, Eigen::Array3d pos, CollisionGrid* gr);
		virtual ~Particle();

		void update(double dt, Eigen::Array3d a);

		Eigen::Array3d get_position();
		Eigen::Array3d get_velocity();
		
		friend std::ostream& operator<<(std::ostream& os, const Particle& p);
	private:

		bool first_step_taken;

		CollisionGrid* grid;
		Sphere S;
		double r;
		Eigen::Vector3d v;
		Eigen::Vector3d x;
		std::vector< Eigen::VectorXd > traj;

};

#endif // PARTICLE_HPP
