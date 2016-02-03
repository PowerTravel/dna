#ifndef MEAN_DISTANCE_HPP
#define MEAN_DISTANCE_HPP

#include "Simulation.hpp"
#include "CollisionGrid.hpp"
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
		bool three_simultaneous_collisions();
		bool plane_four_mixed_collisions_test();
		bool plane_stuck_particle();
};

#endif // MEAN_DISTANCE_HPP
