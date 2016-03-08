#ifndef PARTICLE_HPP
#define PARTICLE_HPP

#include <memory>
#include <random>
#include "CollisionGrid2.hpp"
#include "Sphere.hpp"
#include "Plane.hpp"
#include "Cylinder.hpp"
class Particle{

	public:

		Particle(double dt, double rad, Eigen::Array3d pos,Eigen::Array3d vel, CollisionGrid* gr);
		virtual ~Particle();

		Eigen::ArrayXXd debug_run(double dt, Eigen::Array3d a);

		void update();

		void set_periodic_boundary(VecXd bound);

		Eigen::Array3d get_position();
		Eigen::Array3d get_velocity();
		friend std::ostream& operator<<(std::ostream& os, const Particle& p);
		
		void set_test_collision_vector(std::vector<cg_ptr> v );


		bool use_brownian;
	private:

		struct particle_state
		{
			Eigen::Vector3d pos;
			Eigen::Vector3d vel;
			double dt;	// where inside a timestep we are

		};

		struct debug_snapshot
		{
			int i;					// The index of the vector where the error occured
			std::vector<cg_ptr> v;	// The vector with the to be tested geometries
			CollisionGeometry::coll_struct cs; // the derived collision struct of i
			Vec3d collision_normal;
			Vec3d contact_point;
			double penetration_depth;
			double collision_time;

			particle_state state;	// The state of the particle
			double radius;
		};

		struct collision{
			Vec3d n;
			double t;
		};

		// Subfunctions for update:
		particle_state handle_collisions(particle_state state);
		collision get_earliest_collision( particle_state particle);

		Vec3d reflect_velocity(Vec3d v, Vec3d n);

		Eigen::Vector3d get_random_vector(double min_len, double max_len);

		bool first_step;
		bool use_periodic_boundary;
		VecXd boundary;
		Vec3d boundary_leaps;
		Vec3d boundary_span;
		bool particle_is_stuck;
		CollisionGrid* grid;
		double _r;
		double _dt;
		Eigen::Vector3d _v;
		Eigen::Vector3d _x;
		std::vector< Eigen::VectorXd > traj;
		std::vector< Eigen::VectorXd > collisions_spots;
		
		static std::default_random_engine _generator;
		int timestep = 0;

		// Debug funcs and tests
		std::vector<cg_ptr> test_coll_vec;
		std::vector<cg_ptr > remove_cylinders(std::vector<cg_ptr > vec);

		
		void dump_info( debug_snapshot ds );
};
		

#endif // PARTICLE_HPP
