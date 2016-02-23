#ifndef MEAN_DISTANCE_HPP
#define MEAN_DISTANCE_HPP

#include "Simulation.hpp"
#include "CollisionGrid2.hpp"
#include "Particle.hpp"
class Distance: public Simulation
{
	public:
		Distance();
		Distance(std::map<std::string, std::string> sm);
		virtual ~Distance();
		void apply();

	private:

		CollisionGrid _cg; 
		Eigen::Vector3d _particle_x_ini; 
		Eigen::Vector3d _particle_v_ini;
		void run();
		Eigen::ArrayXXd run_simulation_once();

		void init_simulaition_parameters(std::map<std::string, std::string> sm);
		void print(std::ostream& os);



		Eigen::ArrayXXd P;		// Position
		Eigen::ArrayXXd P_var;	// Position variance
		Eigen::ArrayXd D;		// Distance
		Eigen::ArrayXd D_var;	// Distance Variance


		Eigen::ArrayXd _time_steps;
		Eigen::ArrayXd _time_step_mean;
		
		int _nr_simulations;
		int _nr_data_points;
		int _chain_size;
		double _chain_radius;


		double _collision_box_size;
		double _particle_radius;
		double _tot_time = 100;
		double _dt = 0.01;
		bool _exp = false;
		
		VecXd _boundary;

		int get_max(Eigen::Array3d v);

		void write_to_terminal(int i, int j);
		void write_to_file();

		// Tests 
		void run_box_test(); // A box containing the particle, testing basic collision with plane
		void run_diamond_test(); // A diamond containing the particle testing simultaneous collisions with plane
		void run_sphere_test();

		void run_tests();
		bool plane_one_collision_test_A();
		bool plane_one_collision_test_B();
		bool plane_two_consecutive_collisions_test();
		bool plane_two_simultaneous_collisions_test();
		bool plane_three_simultaneous_collisions();
		bool plane_four_mixed_collisions_test();
//		bool plane_stuck_particle();

		bool sphere_one_collision_test_A();
		bool sphere_one_collision_test_B();
		bool sphere_two_consecutive_collisions_test();
		bool sphere_two_simultaneous_collisions_test();
		bool sphere_three_simultaneous_collisions();
		bool sphere_four_mixed_collisions_test();
//		bool sphere_stuck_particle();

		// Test CollisionGrid
		bool collision_grid_test_one_sphere_A(); // One sphere in one box
		bool collision_grid_test_one_sphere_B(); // One between 4 boxes
		bool collision_grid_test_one_sphere_C(); // One sphere bigger than one box
		bool collision_grid_test_two_spheres_A();// Two spheres in same box
		bool collision_grid_test_two_spheres_B();// Two spheres in two boxes each where they share one
		bool collision_grid_test_two_spheres_C();// Two spheres wich are totally separate
		bool collision_grid_test_many_spheres();
		bool Vec3d_equals(Vec3d a, Vec3d b, double tol = 0);
		bool sphere_test_mirror();

};

#endif // MEAN_DISTANCE_HPP
