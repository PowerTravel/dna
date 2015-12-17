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
		Eigen::Vector3d get_energy(Eigen::Vector3d g);	
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
		double _r;
		Eigen::Vector3d _v;
		Eigen::Vector3d _x;
		std::vector< Eigen::VectorXd > traj;
		std::list< intersections > collisions;
		
		Eigen::Vector3d _E;

		intersections align_normal(intersections is, Eigen::Vector3d v);

		Eigen::VectorXd do_one_collision(double dt_tot, Eigen::VectorXd X, Eigen::Vector3d a, intersections is);

		std::list<intersections> get_coll_list(std::vector<std::shared_ptr< CollisionGeometry> > v, Sphere s);

		std::vector< std::shared_ptr<CollisionGeometry> > build_sphere_and_plane();
};

#endif // PARTICLE_HPP
