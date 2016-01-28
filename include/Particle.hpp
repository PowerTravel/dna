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

		struct intersections{
			intersections()
			{
				geom = NULL;
				composite = false;
				effective_n = Eigen::Vector3d::Zero();
			}
			std::shared_ptr<CollisionGeometry> geom;
			CollisionGeometry::coll_struct cs; 
			Eigen::Vector3d effective_n;
			bool composite;
			void print()
			{	
				std::cerr << "Geom: " << std::endl;
				if(geom != NULL)
				{
					std::cerr <<  geom->get_id() << " - "<< geom->text_type() <<std::endl;
					
				}else{
					std::cerr << "	NULL" <<std::endl;
				}

				std::cerr << "Penetration depth: " << cs.p << std::endl;
				std::cerr << "Collision normal: " << cs.n.transpose() << std::endl;
				std::cerr << "Effective normal: " << effective_n.transpose() << std::endl;
			}
		};


		// Subfunctions for update:
		particle_state do_collisions(particle_state state);
		particle_state do_one_collision(intersections I, particle_state state);
		particle_state get_collision_state(intersections I, particle_state state);


		Eigen::Vector3d get_random_vector(double min_len, double max_len);

		int check_for_simultaneous_collisions(std::vector< intersections > v, particle_state state);


		static bool sort_after_penetration_depth(const Particle::intersections& first, const intersections& second);

		bool first_step;

		CollisionGrid* grid;
		double _r;
		Eigen::Vector3d _v;
		Eigen::Vector3d _x;
		std::vector< Eigen::VectorXd > traj;
	//	std::list< intersections > collisions;
		
		static std::default_random_engine _generator;


		std::vector<intersections> get_coll_vec(std::vector<cg_ptr > v, particle_state particle);

		// Debug funcs and tests
		std::vector<cg_ptr> test_coll_vec;
		std::vector<cg_ptr > remove_cylinders(std::vector<cg_ptr > vec);

		
};
		

#endif // PARTICLE_HPP
