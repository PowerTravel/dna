#include "Distance.hpp"
#include "Statistics.hpp"
#include <iomanip>


Distance::Distance()
{

}
Distance::~Distance()
{

}
Distance::Distance(std::map<std::string, std::string> sm) : Simulation(sm)
{
	_simulation_type = val_map.at("distance");
	init_simulaition_parameters(sm);
}

void Distance::init_simulaition_parameters(std::map<std::string, std::string> sm)
{

	if( sm.find("COLLISION_GRID_BOX_SIZE") != sm.end() )
	{
		_collision_box_size  = text_to_double(sm["COLLISION_GRID_BOX_SIZE"]);
	}else{
		_valid = false;
	}

	if( sm.find("CHAIN_LENGTH") != sm.end() )
	{
		_chain_size  = text_to_int(sm["CHAIN_LENGTH"]);
	}else{
		_valid = false;
	}

	if( sm.find("CHAIN_RADIUS") != sm.end() )
	{
		_chain_radius  = text_to_double(sm["CHAIN_RADIUS"]);
	}else{
		_valid = false;
	}

	if( sm.find("PARTICLE_RADIUS") != sm.end() )
	{
		_particle_radius  = text_to_double(sm["PARTICLE_RADIUS"]);
	}else{
		_valid = false;
	}

	if( sm.find("SIMULATION_TIME") != sm.end() )
	{
		_tot_time  = text_to_double(sm["SIMULATION_TIME"]);
	}else{
		_valid = false;
	}
	
	if( sm.find("DELTA_TIME") != sm.end() )
	{
		_dt  = text_to_double(sm["DELTA_TIME"]);
	}else{
		_valid = false;
	}
	
	if( sm.find("STRIDES") != sm.end() )
	{
		_nr_data_points  = text_to_int(sm["STRIDES"]);
	}else{
		_valid = false;
	}

	if( sm.find("SAMPLES") != sm.end() )
	{
		_nr_simulations  = text_to_int(sm["SAMPLES"]);
	}else{
		_valid = false;
	}

	if( sm.find("EXP") != sm.end() )
	{
		_exp  = text_to_bool(sm["EXP"]);
	}else{
		_valid = false;
	}

}

int Distance::get_max(Eigen::Array3d v)
{
	double max = v(0);
	if( max < v(1) )
	{
		max = v(1);	
	}
	if( max < v(2) )
	{
		max = v(2);	
	}
	return max;
}


void Distance::apply()
{
//	run();
	//run_box_test();
	//run_sphere_test();

	//CollisionGrid::run_tests();
	//run_diamond_test();

	run_tests();
}
void Distance::run()
{
	print_pre_info();
	if(!_valid)
	{
		std::cerr << "Verify not valid. Exiting" << std::endl;
		return;
	}
	if(verbose){
		std::cout << this << std::endl;
	}
	
	_c->set_radius(_chain_radius);
	_c->set_link_length(1.0);
	_c->build(_chain_size);
	_c->center_chain();

	_cg = CollisionGrid(_collision_box_size);
	int  max_axis = get_max(_c->axis_length());
	//_cg.set_up(_c->get_collision_vec() , max_axis );

	_cg.set_up(_c->get_collision_vec() );
	// TODO implement thisi
	//_cg.print_box_corners(std::string("../matlab/Distance/grid.txt"));

	_particle_x_ini = Eigen::Vector3d(0.5, 0.5, 0.5);
	_particle_v_ini = Eigen::Vector3d(0, 0, 0);

	int T = int( _tot_time /_dt);
	int start_point = 10;

	_time_steps = Statistics::make_exponential_points_array(T, _nr_data_points, start_point );
	_time_step_mean = (_time_steps.segment(0, _nr_data_points) + _time_steps.segment(1, _nr_data_points)) / 2;


	Eigen::ArrayXXd binned_distance_data = 
							Eigen::ArrayXXd::Zero(_nr_data_points, 3*_nr_simulations);


	for(int i = 0; i < _nr_simulations; i++)
	{
		Eigen::ArrayXXd trajectory = run_simulation_once();

		for(int j = 0; j<_nr_data_points; j++)
		{
			int start = floor(_time_steps(j));
			int len = floor(_time_steps(j+1) - start);
			if(len == 0)
			{
				len = 1;
			}

			Eigen::ArrayXXd tmp_mat = 
				trajectory.block(0, start, 3, len);
			tmp_mat.row(0) = tmp_mat.row(0) - _particle_x_ini(0);
			tmp_mat.row(1) = tmp_mat.row(1) - _particle_x_ini(1);
			tmp_mat.row(2) = tmp_mat.row(2) - _particle_x_ini(2);

			binned_distance_data(j,3*i+0) = tmp_mat.row(0).mean();
			binned_distance_data(j,3*i+1) = tmp_mat.row(1).mean();	
			binned_distance_data(j,3*i+2) = tmp_mat.row(2).mean();

		}
		if(verbose){
			write_to_terminal(i,_nr_data_points);	
		}
	}

	D = Eigen::ArrayXd::Zero(_nr_data_points);
	D_var = Eigen::ArrayXd::Zero(_nr_data_points);
	Eigen::ArrayXXd D_tmp = Eigen::ArrayXXd::Zero(_nr_data_points, _nr_simulations);
	
	P = Eigen::ArrayXXd::Zero(_nr_data_points, 3);
	P_var = Eigen::ArrayXXd::Zero(_nr_data_points, 3);
	Eigen::ArrayXXd P_tmp_x = Eigen::ArrayXXd::Zero(_nr_data_points, _nr_simulations);
	Eigen::ArrayXXd P_tmp_y = Eigen::ArrayXXd::Zero(_nr_data_points, _nr_simulations);
	Eigen::ArrayXXd P_tmp_z = Eigen::ArrayXXd::Zero(_nr_data_points, _nr_simulations);

	for(int i = 0; i < _nr_data_points; i++)
	{
		for(int j = 0; j < _nr_simulations; j++)
		{
			Eigen::Vector3d pos  = binned_distance_data.block(i,3*j,1,3).transpose();
	
			P_tmp_x(i,j) = pos(0);
			P_tmp_y(i,j) = pos(1);
			P_tmp_z(i,j) = pos(2);

			D_tmp(i,j) = pos.norm();
		}
	}
	

	for(int i = 0; i<_nr_data_points; i++)
	{
	
		Eigen::Vector2d mv;

		mv = Statistics::get_mean_and_variance(P_tmp_x.row(i));
		P(i,0) = mv(0); 
		P_var(i,0) = mv(1);

		mv = Statistics::get_mean_and_variance(P_tmp_y.row(i));
		P(i,1) = mv(0); 
		P_var(i,1) = mv(1);

		mv = Statistics::get_mean_and_variance(P_tmp_z.row(i));
		P(i,2) = mv(0); 
		P_var(i,2) = mv(1);

		mv = Statistics::get_mean_and_variance(D_tmp.row(i));
		D(i) = mv(0);
		D_var(i) = mv(1);
	}

	write_to_file();

	if(verbose)
	{
		print_post_info();
	}
}

void Distance::write_to_terminal(int i, int j)
{
	double p = ( ( double) i*_time_steps(_time_steps.size()-1) + j) / ((double) _nr_simulations*_time_steps(_time_steps.size()-1)) * 100.f;
	std::cout << "\r"<< "["<< std::setw(5)  << std::setprecision(1)<< p << std::fixed << "%] "
			<<"Running..." << std::flush;
}
void Distance::write_to_file()
{
	int idx = _time_steps.size()-1;
	Eigen::MatrixXd result = Eigen::MatrixXd::Zero(idx, 9);

	result.block(0,0,idx,1) = _time_step_mean;
	result.block(0,1,idx,1) = D;
	result.block(0,2,idx,1) = D_var;
	result.block(0,3,idx,3) = P;
	result.block(0,6,idx,3) = P_var;
	
	std::ofstream file;
	file.open(_outfile, std::fstream::out | std::fstream::trunc);
	if(file.is_open()){
		file << result << std::endl;
	}else{
		std::cerr << "Failed to open " << std::string(_outfile) << std::endl;
	}
	file.close();
}


Eigen::ArrayXXd Distance::run_simulation_once()
{
	if(_nr_simulations==1)
	{
		// TODO implement this
		//_cg.print_box_corners(std::string("../matlab/Distance/debug/grid"));
	}
	Particle p = Particle(_dt, _particle_radius, _particle_x_ini, _particle_v_ini, &_cg);
	int N = int(_tot_time/_dt);
	Eigen::ArrayXXd trajectory = Eigen::ArrayXXd::Zero(3,N);
	for(int i = 0; i < N; i++)
	{
		p.update();
		trajectory.block(0,i,3,1) = p.get_position();
	}

	if(_nr_simulations==1)
	{
		std::ofstream traj_file;
		traj_file.open("../matlab/Distance/debug/trajectory", std::fstream::out | std::fstream::trunc);
		if(traj_file.is_open()){
			traj_file << trajectory.transpose() << std::endl;
		}else{
			std::cerr << "../matlab/Distance/debug/trajectory" << std::endl;
		}
		traj_file.close();
		std::cout << "Distance::run_simulation_once: Printed grid and trajectory to '../matlab/Distance/debug/trajectory' and '../matlab/Distance/debug/grid'"  << std::endl;
	}


	return trajectory;
}

void Distance::print(std::ostream& os)
{
	os <<"Simulation = Distance" << std::endl;
	if(_valid)
	{
		os <<"Chain Size = " << _chain_size<< std::endl;
		os <<"Chain Radius = " << _chain_radius << std::endl;

		os <<"Particle Radius = " << _particle_radius << std::endl;
		
		os <<"Collision box-size = " << _collision_box_size << std::endl;

		os <<"Number of Simulations  = " << _nr_simulations << std::endl;
		os <<"Simulation time = " << _tot_time << std::endl;
		os <<"Timestep Size = " << _dt << std::endl;
		
		os <<"Plot points = " << _nr_data_points << std::endl;
		os <<"Exponential = " ;
		if(_exp)
		{
			os << "true" << std::endl;
		}else{
			os << "false" << std::endl;
		}
		os <<"Out File   = " << _outfile << std::endl;
	}else{
		os << "Simulation failed to load.";
	}
};

/*
	Collisions are currently handled in the following way:
		At the start of a timestep a particle is assumed not to be intersecting 
		another geometry.

		1:  move the particle acording to Euler Leap Frog dt forward in time
		2:  do broad phase collision detection, if none detected, save new position
			and return, else move to 3
		3: 	do narrow phase collision detection, if none are detected save new 
			position and return, else move to 4
		4: 	draw check when in the timestep all intersectiond occured and sort them
			in order from earliest collision to latest.
		5: 	handle the earliest collision, the case where the two earliest
			collisions happened simultaneously is an edge case which is handled by
			aligning their collision normals and adding them together then treating
			them as one collision.
		6:  A collision is handled by moving the particle back to the time when
			the collision occured, reflecting velocity around the collision
			normal and forwarding the time back to the time dt.
		7:	Start from 2 again and make sure that the time in step 5 that the 
			time is never reverted further back than the previous pass.
			Time can never be reverted past a handled collision
		8: 	A particle is considered stuck if it cant end it's timestep without 
			having intersections.

		// TODO
		Can we do collisiondetection at the beginning of each timestep instead and 
		always only handle one collision each timestep? 

		// TRY THIS FIRST

	We need four collision tests per geometry:
		1 - Max One registered intersection per timestep
		2 - Two or more registered intersections in a time step
		3 - Stuck particle
			If a particle ends its timestep without being free its 
			counted as bing stuck.
*/


void Distance::run_box_test()
{
	double box_r = 2;
	std::vector< cg_ptr > coll_geom_vec;
	coll_geom_vec.push_back( std::shared_ptr<CollisionGeometry>( 
				new Plane(Vec3d(0,box_r,0), Vec3d(0,1,0)) ));
	coll_geom_vec.push_back( std::shared_ptr<CollisionGeometry>( 
				new Plane(Vec3d(0,-box_r,0), Vec3d(0,-1,0)) ));
			

	// Left and right wall
	coll_geom_vec.push_back( std::shared_ptr<CollisionGeometry>( 
				new Plane(Vec3d(box_r,0,0), Vec3d(1,0,0)) ));
	coll_geom_vec.push_back( std::shared_ptr<CollisionGeometry>( 
				new Plane(Vec3d(-box_r,0,0), Vec3d(-1,0,0)) ));

	// Front back wall
	coll_geom_vec.push_back( std::shared_ptr<CollisionGeometry>( 
				new Plane(Vec3d(0,0,box_r), Vec3d(0,0,1)) ));
	coll_geom_vec.push_back( std::shared_ptr<CollisionGeometry>( 
				new Plane(Vec3d(0,0, -box_r), Vec3d(0,0,-1)) ));

	_particle_x_ini = Eigen::Vector3d(0, 0, 0);

	// One bounce per timestep case:
	//_particle_v_ini = Eigen::Vector3d(1, 0.5, 0.7);
	_particle_v_ini = Eigen::Vector3d(1.01, 1, 1);
	

	Particle p = Particle(_dt, _particle_radius, _particle_x_ini, _particle_v_ini, &_cg);
	p.set_test_collision_vector(coll_geom_vec);

	int N = int(_tot_time/_dt);
	Eigen::ArrayXXd trajectory = Eigen::ArrayXXd::Zero(3,N);
	for(int i = 0; i < N; i++)
	{
		p.update();
		trajectory.block(0,i,3,1) = p.get_position();
	}

	std::ofstream file;
	file.open("../matlab/Distance/particle_trajectory.dna", 
					std::fstream::out | std::fstream::trunc);
	if(file.is_open()){
		file << trajectory.transpose() << std::endl;
	}else{
		std::cerr << "Failed to open " << std::string("../matlab/Distance/particle_trajectory.dna") << std::endl;
	}
	file.close();

}

void Distance::run_diamond_test()
{

	double box_r = 1;
	std::vector< cg_ptr > coll_geom_vec;
	// Top dome
	coll_geom_vec.push_back( std::shared_ptr<CollisionGeometry>( 
				new Plane(Vec3d(box_r,box_r,box_r), Vec3d(-1,-1,-1)) ));
	coll_geom_vec.push_back( std::shared_ptr<CollisionGeometry>( 
				new Plane(Vec3d(box_r,box_r,-box_r), Vec3d(-1,-1,1)) ));
			
	coll_geom_vec.push_back( std::shared_ptr<CollisionGeometry>( 
				new Plane(Vec3d(-box_r,box_r,-box_r), Vec3d(1,-1,1)) ));
	coll_geom_vec.push_back( std::shared_ptr<CollisionGeometry>( 
				new Plane(Vec3d(-box_r,box_r,box_r), Vec3d(-1,1,1)) ));

	// Bottom dome
	coll_geom_vec.push_back( std::shared_ptr<CollisionGeometry>( 
				new Plane(Vec3d(box_r,-box_r,box_r), Vec3d(-1,1,-1)) ));
	coll_geom_vec.push_back( std::shared_ptr<CollisionGeometry>( 
				new Plane(Vec3d(box_r,-box_r,-box_r), Vec3d(-1,1,1)) ));
			
	coll_geom_vec.push_back( std::shared_ptr<CollisionGeometry>( 
				new Plane(Vec3d(-box_r,-box_r,-box_r), Vec3d(1,1,1)) ));
	coll_geom_vec.push_back( std::shared_ptr<CollisionGeometry>( 
				new Plane(Vec3d(-box_r,-box_r,box_r), Vec3d(1,1,-1)) ));
	


	_particle_x_ini = Eigen::Vector3d(0, 0, 0);

	// One bounce per timestep case:
	_particle_v_ini = Eigen::Vector3d(0, 1, 0);
	
	Particle p = Particle(_dt, _particle_radius, _particle_x_ini, _particle_v_ini, &_cg);
	p.set_test_collision_vector(coll_geom_vec);

	int N = int(_tot_time/_dt);
	Eigen::ArrayXXd trajectory = Eigen::ArrayXXd::Zero(3,N);
	for(int i = 0; i < N; i++)
	{
		p.update();
		trajectory.block(0,i,3,1) = p.get_position();
	}

	std::ofstream file;
	file.open("../matlab/Distance/particle_trajectory.dna", 
					std::fstream::out | std::fstream::trunc);
	if(file.is_open()){
		file << trajectory.transpose() << std::endl;
	}else{
		std::cerr << "Failed to open " << std::string("../matlab/Distance/particle_trajectory.dna") << std::endl;
	}
	file.close();
}

void Distance::run_sphere_test()
{
	double sphere_r = _chain_radius;
	double particle_r = _particle_radius;
	std::vector< cg_ptr > coll_geom_vec;
	int I = 10;
	int J = 10;
	int K = 10;
	for(int i=-I; i<=I; i++)
	{
		for(int j=-J; j<=J; j++)
		{
			for(int k=-K; k<=K; k++)
			{
				coll_geom_vec.push_back( std::shared_ptr<CollisionGeometry>( 
				new Sphere(Vec3d(i,j,k), sphere_r) ));	
			}	
		}
	}
 	
	_cg = CollisionGrid(_collision_box_size);
	//_cg.set_up(coll_geom_vec , I );


	_cg.set_up(coll_geom_vec );
	// TODO implement this
	// _cg.print_box_corners(std::string("../matlab/Distance/debug/grid"));

	_particle_x_ini = Eigen::Vector3d(0.5, 0.5, 0.5);
	_particle_v_ini = Eigen::Vector3d(0.01, 0.1, 0.04);
	
	Particle p = Particle(_dt, particle_r, _particle_x_ini, _particle_v_ini, &_cg);
	//Particle p = Particle(_dt, particle_r, _particle_x_ini, _particle_v_ini, NULL);
	//p.set_test_collision_vector(coll_geom_vec);

	int N = int(_tot_time/_dt);
	Eigen::ArrayXXd trajectory = Eigen::ArrayXXd::Zero(3,N);
	for(int i = 0; i < N; i++)
	{	
		p.update();
		trajectory.block(0,i,3,1) = p.get_position();
	}

	std::ofstream file;
	file.open("../matlab/Distance/debug/trajectory", 
					std::fstream::out | std::fstream::trunc);
	if(file.is_open()){
		std::cout << "Distance::run_sphere_test:" << std::endl;
		std::cout << "	Writing to '../matlab/Distance/debug/trajectory'" << std::endl;
		file << trajectory.transpose() << std::endl;
	}else{
		std::cerr << "Failed to open " << std::string("../matlab/Distance/particle_trajectory.dna") << std::endl;
	}
	file.close();
}



void Distance::run_tests()
{

	std::cerr << "TEST plane_one_collision_test_A ";
	if(plane_one_collision_test_A())
	{
		 std::cerr <<"SUCCEEDED" << std::endl;
	}else{
		 std::cerr <<"FAILED" << std::endl;
	}
	
	std::cerr << "TEST plane_one_collision_test_B ";
	if(plane_one_collision_test_B())
	{
		 std::cerr <<"SUCCEEDED" << std::endl;
	}else{
		 std::cerr <<"FAILED" << std::endl;
	}

	std::cerr << "TEST plane_two_consecutive_collisions_test ";
	if(plane_two_consecutive_collisions_test())
	{
		 std::cerr <<"SUCCEEDED" << std::endl;
	}else{
		 std::cerr <<"FAILED" << std::endl;
	}
	
	std::cerr << "TEST plane_two_simultaneous_collisions_test ";
	if(plane_two_simultaneous_collisions_test())
	{
		 std::cerr <<"SUCCEEDED" << std::endl;
	}else{
		 std::cerr <<"FAILED" << std::endl;
	}
	
	std::cerr << "TEST plane_three_simultaneous_collisions ";
	if(plane_three_simultaneous_collisions())
	{
		 std::cerr <<"SUCCEEDED" << std::endl;
	}else{
		 std::cerr <<"FAILED" << std::endl;
	}

	std::cerr << "TEST plane_four_mixed_collisions_test ";
	if(plane_four_mixed_collisions_test())
	{
		 std::cerr <<"SUCCEEDED" << std::endl;
	}else{
		 std::cerr <<"FAILED" << std::endl;
	}

	std::cerr << "TEST sphere_one_collision_test_A ";
	if(sphere_one_collision_test_A())
	{
		 std::cerr <<"SUCCEEDED" << std::endl;
	}else{
		 std::cerr <<"FAILED" << std::endl;
	}
	
	std::cerr << "TEST sphere_one_collision_test_B ";
	if(sphere_one_collision_test_B())
	{
		 std::cerr <<"SUCCEEDED" << std::endl;
	}else{
		 std::cerr <<"FAILED" << std::endl;
	}

	std::cerr << "TEST sphere_two_consecutive_collisions_test ";
	if(sphere_two_consecutive_collisions_test())
	{
		 std::cerr <<"SUCCEEDED" << std::endl;
	}else{
		 std::cerr <<"FAILED" << std::endl;
	}
	
	std::cerr << "TEST sphere_two_simultaneous_collisions_test ";
	if(sphere_two_simultaneous_collisions_test())
	{
		 std::cerr <<"SUCCEEDED" << std::endl;
	}else{
		 std::cerr <<"FAILED" << std::endl;
	}
	
	std::cerr << "TEST sphere_three_simultaneous_collisions ";
	if(sphere_three_simultaneous_collisions())
	{
		 std::cerr <<"SUCCEEDED" << std::endl;
	}else{
		 std::cerr <<"FAILED" << std::endl;
	}

	std::cerr << "TEST sphere_four_mixed_collisions_test ";
	if(sphere_four_mixed_collisions_test())
	{
		 std::cerr <<"SUCCEEDED" << std::endl;
	}else{
		 std::cerr <<"FAILED" << std::endl;
	}





}

bool Distance::plane_one_collision_test_A()
{
	double tol = 0.000000000001;
	// Collision happens on the boundary of update one and two.
	// at t = dt
	Vec3d particle_position = Vec3d(0,0.4,0);
	Vec3d particle_velocity = Vec3d(1,1,0);
	double particle_radie = 0.5;
	double dt = 0.1;
	
	Vec3d plane_point = Vec3d(0,1,0); 
	Vec3d plane_normal = Vec3d(0,1,0);
	
	Particle particle = Particle(dt, particle_radie, 
									 particle_position, 
									 particle_velocity, NULL);
	particle.use_brownian=false;

	std::vector<cg_ptr> v = std::vector<cg_ptr>();
	v.push_back(cg_ptr( new Plane(plane_point, plane_normal) ) );	
	particle.set_test_collision_vector(v); 
	
	particle.update(); // No collision should be detected
	particle.update(); // Collision should be detected and particle should be 
					   // reflected
	Vec3d p_prim = particle.get_position();
	Vec3d v_prim = particle.get_velocity();

	// What values should be
	Vec3d post_collision_position(0.2,0.4,0);
	Vec3d post_collision_velocity(1,-1,0);

	if( std::abs(post_collision_position(0) - p_prim(0)) >= tol ||
		std::abs(post_collision_position(1) - p_prim(1)) >= tol ||
		std::abs(post_collision_position(2) - p_prim(2)) >= tol ||
		std::abs(post_collision_velocity(0) - v_prim(0)) >= tol ||
		std::abs(post_collision_velocity(1) - v_prim(1)) >= tol ||
		std::abs(post_collision_velocity(2) - v_prim(2)) >= tol )
	{
		std::cerr << p_prim.transpose() <<std::endl;
		std::cerr << v_prim.transpose() <<std::endl;
		return false;	
	}
	return true;
}

bool Distance::plane_one_collision_test_B()
{
	double tol = 0.000000000001;

	// Collision happens at t=dt/2
	Vec3d particle_position = Vec3d(0,0.35,0);
	Vec3d particle_velocity = Vec3d(1,1,0);
	double particle_radie = 0.5;
	double dt = 0.1;
	
	Vec3d plane_point = Vec3d(0,1,0); 
	Vec3d plane_normal = Vec3d(0,-1,0);
	
	Particle particle = Particle(dt, particle_radie, 
									 particle_position, 
									 particle_velocity, NULL);
	particle.use_brownian=false;

	std::vector<cg_ptr> v = std::vector<cg_ptr>();
	v.push_back(cg_ptr( new Plane(plane_point, plane_normal) ) );	
	particle.set_test_collision_vector(v); 
	
	particle.update(); // No collision should be detected
	particle.update(); // Collision should be detected and particle should be 
					   // reflected
	Vec3d p_prim = particle.get_position();
	Vec3d v_prim = particle.get_velocity();

	// What values should be
	Vec3d post_collision_position(0.2,0.45,0);
	Vec3d post_collision_velocity(1,-1,0);

	if( std::abs(post_collision_position(0) - p_prim(0)) >= tol ||
		std::abs(post_collision_position(1) - p_prim(1)) >= tol ||
		std::abs(post_collision_position(2) - p_prim(2)) >= tol ||
		std::abs(post_collision_velocity(0) - v_prim(0)) >= tol ||
		std::abs(post_collision_velocity(1) - v_prim(1)) >= tol ||
		std::abs(post_collision_velocity(2) - v_prim(2)) >= tol )
	{
		std::cerr << p_prim.transpose() << std::endl;
		std::cerr << v_prim.transpose() << std::endl;
		return false;	
	}
	return true;
}

bool Distance::plane_two_consecutive_collisions_test()
{
	// Orthogonal collisions within a timestep
	double tol = 0.000000000001;

	// Collision happens at t=dt/2
	Vec3d particle_position = Vec3d(0,0,0);
	Vec3d particle_velocity = Vec3d(1,1,0);
	double particle_radie = 0.5;
	double dt = 0.1;
	
	Particle particle = Particle(dt, particle_radie, 
									 particle_position, 
									 particle_velocity, NULL);

	particle.use_brownian=false;

	std::vector<cg_ptr> v = std::vector<cg_ptr>();
	Vec3d plane_point_1 = Vec3d(0.52,0,0); 
	Vec3d plane_normal_1 = Vec3d(1,0,0);
	v.push_back(cg_ptr( new Plane(plane_point_1, plane_normal_1) ) );	

	Vec3d plane_point_2 = Vec3d(0,0.58,0); 
	Vec3d plane_normal_2 = Vec3d(0,-1,0);
	v.push_back(cg_ptr( new Plane(plane_point_2, plane_normal_2) ) );	

	particle.set_test_collision_vector(v); 
	
	particle.update(); // Collision should happen with plane 1 at t = 0.02
					   // Collision should happen with plane 2 at t = 0.08 

	Vec3d p_prim = particle.get_position();
	Vec3d v_prim = particle.get_velocity();

	// What values should be
	Vec3d post_collision_position(-0.06, 0.06 ,0);
	Vec3d post_collision_velocity(-1,-1,0);

	if( std::abs(post_collision_position(0) - p_prim(0)) >= tol ||
		std::abs(post_collision_position(1) - p_prim(1)) >= tol ||
		std::abs(post_collision_position(2) - p_prim(2)) >= tol ||
		std::abs(post_collision_velocity(0) - v_prim(0)) >= tol ||
		std::abs(post_collision_velocity(1) - v_prim(1)) >= tol ||
		std::abs(post_collision_velocity(2) - v_prim(2)) >= tol )
	{
		std::cerr << p_prim.transpose() << std::endl;
		std::cerr << v_prim.transpose() << std::endl;
		return false;	
	}
	return true;
}
bool Distance::plane_two_simultaneous_collisions_test()
{
	// Simultaneous Orthogonal collisions within a timestep
	double tol = 0.000000000001;

	// Collision happens at t=dt/2
	Vec3d particle_position = Vec3d(0,0,0);
	Vec3d particle_velocity = Vec3d(1,1,0);
	double particle_radie = 0.5;
	double dt = 0.1;
	
	Particle particle = Particle(dt, particle_radie, 
									 particle_position, 
									 particle_velocity, NULL);
	particle.use_brownian=false;


	std::vector<cg_ptr> v = std::vector<cg_ptr>();
	Vec3d plane_point_1 = Vec3d(0.55,0,0); 
	Vec3d plane_normal_1 = Vec3d(1,0,0);
	v.push_back(cg_ptr( new Plane(plane_point_1, plane_normal_1) ) );	

	Vec3d plane_point_2 = Vec3d(0,0.55,0); 
	Vec3d plane_normal_2 = Vec3d(0,-1,0);
	v.push_back(cg_ptr( new Plane(plane_point_2, plane_normal_2) ) );	

	particle.set_test_collision_vector(v); 
	
	particle.update(); // Collision should happen with plane 1 at t = 0.05
					   // Collision should happen with plane 2 at t = 0.05 

	Vec3d p_prim = particle.get_position();
	Vec3d v_prim = particle.get_velocity();

	// What values should be
	Vec3d post_collision_position(0, 0 ,0);
	Vec3d post_collision_velocity(-1,-1,0);

	if( std::abs(post_collision_position(0) - p_prim(0)) >= tol ||
		std::abs(post_collision_position(1) - p_prim(1)) >= tol ||
		std::abs(post_collision_position(2) - p_prim(2)) >= tol ||
		std::abs(post_collision_velocity(0) - v_prim(0)) >= tol ||
		std::abs(post_collision_velocity(1) - v_prim(1)) >= tol ||
		std::abs(post_collision_velocity(2) - v_prim(2)) >= tol )
	{
		std::cerr << p_prim.transpose() << std::endl;
		std::cerr << v_prim.transpose() << std::endl;
		return false;	
	}
	return true;
}

bool Distance::plane_three_simultaneous_collisions()
{
	//  four collisions within a timestep, two which are simultaneous
	double tol = 0.000000000001;

	// Collision happens at t=dt/2
	Vec3d particle_position = Vec3d(0,0,0);
	Vec3d particle_velocity = Vec3d(1,1,0);
	double particle_radie = std::sqrt(2);
	double dt = 0.1;
	
	Particle particle = Particle(dt, particle_radie, 
									 particle_position, 
									 particle_velocity, NULL);
	particle.use_brownian=false;


	// Plane 1 and 2 make up the simultaneous collisions.
	// They make up a slanted roof
	std::vector<cg_ptr> v = std::vector<cg_ptr>();
	Vec3d plane_point_1 = Vec3d(0,1.05,1); 
	Vec3d plane_normal_1 = Vec3d(0,-1,-1);
	v.push_back(cg_ptr( new Plane(plane_point_1, plane_normal_1) ) );	

	Vec3d plane_point_2 = Vec3d(0,1.05,-1); 
	Vec3d plane_normal_2 = Vec3d(0,-1,1);
	v.push_back(cg_ptr( new Plane(plane_point_2, plane_normal_2) ) );	

	// A flat roof that will register as a collision but should never
	// be evaluated as plane_1 and plane_2 will collide before
	Vec3d plane_point_3 = Vec3d(0,std::sqrt(2)+0.05,0); 
	Vec3d plane_normal_3 = Vec3d(0,-1,0);
	v.push_back(cg_ptr( new Plane(plane_point_3, plane_normal_3) ) );	

	// a wall
	Vec3d plane_point_4 = Vec3d(std::sqrt(2) + 0.05,0,0); 
	Vec3d plane_normal_4 = Vec3d(-1,0,0);
	v.push_back(cg_ptr( new Plane(plane_point_4, plane_normal_4) ) );	

	particle.set_test_collision_vector(v); 
	
	particle.update(); // Collision should happen with plane 4 at t = 0.02
					   // Collision should happen with plane 1 and 2 at t = 0.02 
					   // Collision should never happen with plane 3

	Vec3d p_prim = particle.get_position();
	Vec3d v_prim = particle.get_velocity();

	// What values should be
	Vec3d post_collision_position( 0, 0 ,0);
	Vec3d post_collision_velocity(-1,-1,0);

	if( std::abs(post_collision_position(0) - p_prim(0)) >= tol ||
		std::abs(post_collision_position(1) - p_prim(1)) >= tol ||
		std::abs(post_collision_position(2) - p_prim(2)) >= tol ||
		std::abs(post_collision_velocity(0) - v_prim(0)) >= tol ||
		std::abs(post_collision_velocity(1) - v_prim(1)) >= tol ||
		std::abs(post_collision_velocity(2) - v_prim(2)) >= tol )
	{
		std::cerr << p_prim.transpose() << std::endl;
		std::cerr << v_prim.transpose() << std::endl;
		return false;
	}
	return true;

}
bool Distance::plane_four_mixed_collisions_test()
{
	//  four collisions within a timestep, two which are simultaneous
	double tol = 0.000000000001;

	// Collision happens at t=dt/2
	Vec3d particle_position = Vec3d(0,0,0);
	Vec3d particle_velocity = Vec3d(1,1,0);
	double particle_radie = std::sqrt(2);
	double dt = 0.1;
	
	Particle particle = Particle(dt, particle_radie, 
									 particle_position, 
									 particle_velocity, NULL);
	particle.use_brownian=false;


	// Plane 1 and 2 make up the simultaneous collisions.
	// They make up a slanted roof
	std::vector<cg_ptr> v = std::vector<cg_ptr>();
	Vec3d plane_point_1 = Vec3d(0,1.05,1); 
	Vec3d plane_normal_1 = Vec3d(0,-1,-1);
	v.push_back(cg_ptr( new Plane(plane_point_1, plane_normal_1) ) );	

	Vec3d plane_point_2 = Vec3d(0,1.05,-1); 
	Vec3d plane_normal_2 = Vec3d(0,-1,1);
	v.push_back(cg_ptr( new Plane(plane_point_2, plane_normal_2) ) );	

	// A flat roof that will register as a collision but should never
	// be evaluated as plane_1 and plane_2 will collide before
	Vec3d plane_point_3 = Vec3d(0,particle_radie+0.06,0); 
	Vec3d plane_normal_3 = Vec3d(0,-1,0);
	v.push_back(cg_ptr( new Plane(plane_point_3, plane_normal_3) ) );	

	// a wall
	Vec3d plane_point_4 = Vec3d(particle_radie + 0.02,0,0); 
	Vec3d plane_normal_4 = Vec3d(-1,0,0);
	v.push_back(cg_ptr( new Plane(plane_point_4, plane_normal_4) ) );	

	particle.set_test_collision_vector(v); 
	
	particle.update(); // Collision should happen with plane 4 at t = 0.02
					   // Collision should happen with plane 1 and 2 at t = 0.02 
					   // Collision should never happen with plane 3

	Vec3d p_prim = particle.get_position();
	Vec3d v_prim = particle.get_velocity();

	// What values should be
	Vec3d post_collision_position( -0.06, 0 ,0);
	Vec3d post_collision_velocity(-1,-1,0);

	if( std::abs(post_collision_position(0) - p_prim(0)) >= tol ||
		std::abs(post_collision_position(1) - p_prim(1)) >= tol ||
		std::abs(post_collision_position(2) - p_prim(2)) >= tol ||
		std::abs(post_collision_velocity(0) - v_prim(0)) >= tol ||
		std::abs(post_collision_velocity(1) - v_prim(1)) >= tol ||
		std::abs(post_collision_velocity(2) - v_prim(2)) >= tol )
	{
		std::cerr << p_prim.transpose() << std::endl;
		std::cerr << v_prim.transpose() << std::endl;
		return false;
	}
	return true;
}

bool Distance::sphere_one_collision_test_A()
{
	//  four collisions within a timestep, two which are simultaneous
	// NOTE: This test fails because an error when calculating the collision -normal
	// 	which causes an error when calculationg first point of contact on the sphere
	//  Since the collision normal is position dependent in sphere-sphere collisions
	//	the angle at the end of the dt time step is different from the time between
	//  0 and dt when the collision actually occured.
	// 	Maybe one can use the penetration depth and velocity vector to find the point of first contact 
	//	without using the collision normal and then when after "rewinding the time" // 	calculating the collision normal when the two bodies are staged for 
	//  collision. 
	// 	However since the error is of size dt and we already have alot of randomness
	// 	in the simulation we can let it be for now.

	double tol = 0.000000000001;

	Vec3d particle_position = Vec3d(0,0,0);
	Vec3d particle_velocity = Vec3d(0,1,0);
	double particle_radie = std::sqrt(2);
	double dt = 0.1;

	Particle particle = Particle(dt, particle_radie, 
									 particle_position, 
									 particle_velocity, NULL);
	particle.use_brownian=false;



	std::vector<cg_ptr> v = std::vector<cg_ptr>();
	double sphere_radie = std::sqrt(2);
	Vec3d sphere_point  = Vec3d(2,2,0);
	v.push_back(cg_ptr( new Sphere(sphere_point, sphere_radie) ) );	

	particle.set_test_collision_vector(v); 
	
	particle.update();

	Vec3d p_prim = particle.get_position();
	Vec3d v_prim = particle.get_velocity();

	// What values should be
	Vec3d post_collision_position( -0.1, 0 ,0);
	Vec3d post_collision_velocity(-1,0,0);

	if( std::abs(post_collision_position(0) - p_prim(0)) >= tol ||
		std::abs(post_collision_position(1) - p_prim(1)) >= tol ||
		std::abs(post_collision_position(2) - p_prim(2)) >= tol ||
		std::abs(post_collision_velocity(0) - v_prim(0)) >= tol ||
		std::abs(post_collision_velocity(1) - v_prim(1)) >= tol ||
		std::abs(post_collision_velocity(2) - v_prim(2)) >= tol )
	{
		std::cerr << std::endl << "This simulation fails because of a small error (of size dt) having to do when we calculate collision normal in sphere-sphere collision which is position dependant (which has implications for first point of contact calculations). See comments in function definition Distance::sphere_one_collision_test_A()" << std::endl<< "Deferring fixing untill later since it's not a critical error." <<std::endl;

		std::cerr << p_prim.transpose() << std::endl;
		std::cerr << v_prim.transpose() << std::endl;
			
		return false;
	}
	return true;
}
bool Distance::sphere_one_collision_test_B()
{
	double tol = 0.000000000001;

	Vec3d particle_position = Vec3d(0,0,0);
	Vec3d particle_velocity = Vec3d(1,1,0);
	double particle_radie = std::sqrt(2);
	double dt = 0.1;

	Particle particle = Particle(dt, particle_radie, 
									 particle_position, 
									 particle_velocity, NULL);
	particle.use_brownian=false;



	std::vector<cg_ptr> v = std::vector<cg_ptr>();
	double sphere_radie = std::sqrt(2);
	Vec3d sphere_point  = Vec3d(2.05,2.05,0);
	v.push_back(cg_ptr( new Sphere(sphere_point, sphere_radie) ) );	

	particle.set_test_collision_vector(v); 
	
	particle.update();

	Vec3d p_prim = particle.get_position();
	Vec3d v_prim = particle.get_velocity();

	// What values should be
	Vec3d post_collision_position( 0, 0 ,0);
	Vec3d post_collision_velocity(-1,-1,0);

	if( std::abs(post_collision_position(0) - p_prim(0)) >= tol ||
		std::abs(post_collision_position(1) - p_prim(1)) >= tol ||
		std::abs(post_collision_position(2) - p_prim(2)) >= tol ||
		std::abs(post_collision_velocity(0) - v_prim(0)) >= tol ||
		std::abs(post_collision_velocity(1) - v_prim(1)) >= tol ||
		std::abs(post_collision_velocity(2) - v_prim(2)) >= tol )
	{

		std::cerr << p_prim.transpose() << std::endl;
		std::cerr << v_prim.transpose() << std::endl;
		return false;
	}
	return true;
}
bool Distance::sphere_two_consecutive_collisions_test()
{
	double tol = 0.000000000001;

	Vec3d particle_position = Vec3d(0,0,0);
	Vec3d particle_velocity = Vec3d(1,1,0);
	double particle_radie = std::sqrt(2);
	double dt = 0.1;

	Particle particle = Particle(dt, particle_radie, 
									 particle_position, 
									 particle_velocity, NULL);
	particle.use_brownian=false;



	std::vector<cg_ptr> v = std::vector<cg_ptr>();
	double sphere_radie = std::sqrt(2);
	Vec3d sphere_point  = Vec3d(2.05,2.05,0);
	v.push_back(cg_ptr( new Sphere(sphere_point, sphere_radie) ) );	

	particle.set_test_collision_vector(v); 
	
	particle.update();

	Vec3d p_prim = particle.get_position();
	Vec3d v_prim = particle.get_velocity();

	// What values should be
	Vec3d post_collision_position( 0, 0 ,0);
	Vec3d post_collision_velocity(-1,-1,0);

	if( std::abs(post_collision_position(0) - p_prim(0)) >= tol ||
		std::abs(post_collision_position(1) - p_prim(1)) >= tol ||
		std::abs(post_collision_position(2) - p_prim(2)) >= tol ||
		std::abs(post_collision_velocity(0) - v_prim(0)) >= tol ||
		std::abs(post_collision_velocity(1) - v_prim(1)) >= tol ||
		std::abs(post_collision_velocity(2) - v_prim(2)) >= tol )
	{

		std::cerr << p_prim.transpose() << std::endl;
		std::cerr << v_prim.transpose() << std::endl;
		return false;
	}
	return true;
}
bool Distance::sphere_two_simultaneous_collisions_test()
{
	return false;
}
bool Distance::sphere_three_simultaneous_collisions()
{
	return false;
}
bool Distance::sphere_four_mixed_collisions_test()
{
	return false;
}


