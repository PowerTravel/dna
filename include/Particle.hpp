#ifndef PARTICLE_HPP
#define PARTICLE_HPP

#include <memory>
#include <list>
#include <random>
#include "CollisionGrid.hpp"
#include "Sphere.hpp"
#include "Plane.hpp"
#include "Cylinder.hpp"
class Particle{

	public:
		Particle(double rad, Eigen::Array3d pos,Eigen::Array3d vel, CollisionGrid* gr);
		virtual ~Particle();

		Eigen::ArrayXXd debug_run(double dt, Eigen::Array3d a);


		void update(double dt);

		Eigen::Array3d get_position();
		Eigen::Array3d get_velocity();
		Eigen::Vector3d get_energy(Eigen::Vector3d g);
		friend std::ostream& operator<<(std::ostream& os, const Particle& p);

		
	private:

		struct particle_state
		{
			double dt;		// where inside a timestep we are 
			Eigen::Vector3d pos;
			Eigen::Vector3d vel;
		};

		struct intersections{
			intersections()
			{
				geom = NULL;
				effective_n = Eigen::Vector3d::Zero();
			}
			std::shared_ptr<CollisionGeometry> geom;
			CollisionGeometry::coll_struct cs; 
			Eigen::Vector3d effective_n;
		};

		particle_state do_collisions(particle_state state);
		
		Eigen::Vector3d get_random_vector(double min_len, double max_len);



		static bool sort_after_penetration_depth(const Particle::intersections& first, const intersections& second);

		bool first_step;

		CollisionGrid* grid;
		double _r;
		Eigen::Vector3d _v;
		Eigen::Vector3d _x;
		std::vector< Eigen::VectorXd > traj;
	//	std::list< intersections > collisions;
		
		static std::default_random_engine _generator;

		intersections align_normal(intersections is, Eigen::Vector3d v);

		particle_state do_one_collision(intersections I, particle_state state);
		Eigen::VectorXd do_one_collision(double dt_tot, Eigen::VectorXd X, Eigen::Vector3d a, intersections is);

		std::vector<intersections> get_coll_vec(std::vector<cg_ptr > v, particle_state particle);

		std::vector< std::shared_ptr<CollisionGeometry> > build_sphere_and_plane();

		// C
		std::vector< cg_ptr > build_plane_test();
		
};
		

#endif // PARTICLE_HPP
