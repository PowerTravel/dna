#ifndef PARTICLE_HPP
#define PARTICLE_HPP

#include <memory>
#include <list>
#include "CollisionGrid.hpp"
#include "Sphere.hpp"
#include "Plane.hpp"
class Particle{

	public:
		Particle(double rad, Eigen::Array3d pos,Eigen::Array3d vel, CollisionGrid* gr);
		virtual ~Particle();

		void update(double dt, Eigen::Array3d a);

		Eigen::Array3d get_position();
		Eigen::Array3d get_velocity();
		
		friend std::ostream& operator<<(std::ostream& os, const Particle& p);
	private:

		struct intersections{
			intersections()
			{
				geom = NULL;
			}
			std::shared_ptr<CollisionGeometry> geom;
			CollisionGeometry::coll_struct cs; 
		};


		static bool sort_after_penetration_depth(const Particle::intersections& first, const intersections& second);

		bool first_step_taken;

		CollisionGrid* grid;
		double r;
		Eigen::Vector3d v;
		Eigen::Vector3d x;
		std::vector< Eigen::VectorXd > traj;
		std::list< intersections > collisions;

		void do_one_collision(intersections is);

		std::vector< std::shared_ptr<CollisionGeometry> > build_sphere_and_plane();
};

#endif // PARTICLE_HPP
