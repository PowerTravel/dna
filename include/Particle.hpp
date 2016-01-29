#ifndef PARTICLE_HPP
#define PARTICLE_HPP

#include <memory>
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
		friend std::ostream& operator<<(std::ostream& os, const Particle& p);
		
		void set_test_collision_vector(std::vector<cg_ptr> v );
		
	private:

		struct particle_state
		{
			double dt;		// where inside a timestep we are 
			Eigen::Vector3d pos;
			Eigen::Vector3d vel;

		};

		struct collision{
			Vec3d n;
			double t;
			
		};

		// Subfunctions for update:
		particle_state handle_collisions(particle_state state);
		collision get_earliest_collision(std::vector<cg_ptr > v, particle_state particle);

		Eigen::Vector3d get_random_vector(double min_len, double max_len);

		bool first_step;

		CollisionGrid* grid;
		double _r;
		Eigen::Vector3d _v;
		Eigen::Vector3d _x;
		std::vector< Eigen::VectorXd > traj;
		
		static std::default_random_engine _generator;


		// Debug funcs and tests
		std::vector<cg_ptr> test_coll_vec;
		std::vector<cg_ptr > remove_cylinders(std::vector<cg_ptr > vec);

		
};
		

#endif // PARTICLE_HPP
