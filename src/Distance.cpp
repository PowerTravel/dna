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

	if( sm.find("PARTICLE_RADIUS_MIN") != sm.end() )
	{
		_particle_radius_min  = text_to_double(sm["PARTICLE_RADIUS_MIN"]);
	}else{
		_valid = false;
	}

	if( sm.find("PARTICLE_RADIUS_MAX") != sm.end() )
	{
		_particle_radius_max  = text_to_double(sm["PARTICLE_RADIUS_MAX"]);
	}else{
		_valid = false;
	}

	if( sm.find("PARTICLE_RADIUS_STEP") != sm.end() )
	{
		_particle_radius_step  = text_to_int(sm["PARTICLE_RADIUS_STEP"]);
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
	
	if( (_particle_radius_min <= 0) || (_particle_radius_min >_particle_radius_max) || (_particle_radius_step<= 0))
	{
		_valid = false;
	}
	
	if(_valid == false)
	{
		std::cerr << "ERROR: Distance::init_simulaition_parameters"<<std::endl;
		std::cerr << "	Invalid simulation parameters" <<std::endl;
		std::cerr << "	EXITITNG" <<std::endl;
		exit(1);
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
	run();

	//sphere_test_mirror();

	//run_sphere_test();

	//CollisionGrid::run_tests();

	//run_tests();
	//run_cylinder_test();
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


	int T = int( _tot_time/_dt);
	int start_point = 10;

	if(_exp){
		_time_steps = Statistics::make_exponential_points_array(T, _nr_data_points, start_point );
	}else{
		_time_steps = Statistics::make_linear_points_array(T, _nr_data_points, start_point );
	}
	_time_step_mean = (_time_steps.segment(0, _nr_data_points) + _time_steps.segment(1, _nr_data_points)) / 2;
	
	Eigen::ArrayXXd binned_distance_data = 
							Eigen::ArrayXXd::Zero(_nr_data_points * _particle_radius_step, 3*_nr_simulations);

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

			for( int p_step = 0; p_step < _particle_radius_step; p_step++)
			{
				Eigen::ArrayXXd tmp_mat = 
					trajectory.block(3*p_step, start, 3, len);
				tmp_mat.row(0) = tmp_mat.row(0);
				tmp_mat.row(1) = tmp_mat.row(1);
				tmp_mat.row(2) = tmp_mat.row(2);
				
	//			std::cerr << j+p_step*_nr_data_points << " " << 3*i+0 << std::endl;
					
				binned_distance_data(j+p_step*_nr_data_points,3*i+0) = tmp_mat.row(0).mean();
				binned_distance_data(j+p_step*_nr_data_points,3*i+1) = tmp_mat.row(1).mean();	
				binned_distance_data(j+p_step*_nr_data_points,3*i+2) = tmp_mat.row(2).mean();
			}
		}
		if(verbose){
			write_to_terminal(i,_nr_data_points);	
		}
	}

//	std::cout << binned_distance_data << std::endl;

	D = Eigen::ArrayXd::Zero(_nr_data_points * _particle_radius_step);
	D_var = Eigen::ArrayXd::Zero(_nr_data_points * _particle_radius_step);
	
	Eigen::ArrayXXd D_tmp = Eigen::ArrayXXd::Zero(_nr_data_points*_particle_radius_step, _nr_simulations);
	
	P = Eigen::ArrayXXd::Zero(_nr_data_points*_particle_radius_step, 3);
	P_var = Eigen::ArrayXXd::Zero(_nr_data_points*_particle_radius_step, 3);
	
	Eigen::ArrayXXd P_tmp_x = Eigen::ArrayXXd::Zero(_nr_data_points*_particle_radius_step, _nr_simulations);
	Eigen::ArrayXXd P_tmp_y = Eigen::ArrayXXd::Zero(_nr_data_points*_particle_radius_step, _nr_simulations);
	Eigen::ArrayXXd P_tmp_z = Eigen::ArrayXXd::Zero(_nr_data_points*_particle_radius_step, _nr_simulations);
	
	for(int p_step = 0; p_step < _particle_radius_step; p_step++)
	{
		for(int i = 0; i < _nr_data_points; i++)
		{
			int sim_stride = p_step*_nr_data_points;
			for(int j = 0; j < _nr_simulations; j++)
			{
				Eigen::Vector3d pos = binned_distance_data.block(sim_stride+i,3*j,1,3).transpose();

				P_tmp_x(sim_stride+i,j) = pos(0);
				P_tmp_y(sim_stride+i,j) = pos(1);
				P_tmp_z(sim_stride+i,j) = pos(2);

				D_tmp(sim_stride+i,j) = pos.norm();
			}
		}
	}
	
	for(int p_step = 0; p_step < _particle_radius_step; p_step++)
	{
		for(int i = 0; i<_nr_data_points; i++)
		{
			int sim_stride = p_step*_nr_data_points;
			Eigen::Vector2d mv;

			mv = Statistics::get_mean_and_variance(P_tmp_x.row(sim_stride+i));
			P(sim_stride+i,0) = mv(0); 
			P_var(sim_stride+i,0) = mv(1);

			mv = Statistics::get_mean_and_variance(P_tmp_y.row(sim_stride+i));
			P(sim_stride+i,1) = mv(0); 
			P_var(sim_stride+i,1) = mv(1);

			mv = Statistics::get_mean_and_variance(P_tmp_z.row(sim_stride+i));
			P(sim_stride+i,2) = mv(0); 
			P_var(sim_stride+i,2) = mv(1);

			mv = Statistics::get_mean_and_variance(D_tmp.row(sim_stride+i));
			D(sim_stride+i) = mv(0);
			D_var(sim_stride+i) = mv(1);
		}
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
	int idx = (_time_steps.size()-1) * _particle_radius_step;
	Eigen::MatrixXd result = Eigen::MatrixXd::Zero(idx, 9);
	for(int i=0; i<_particle_radius_step; i++)
	{
		result.block(i*_time_step_mean.size(),0,_time_step_mean.size(),1) = _time_step_mean*_dt;
	}
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
	int N = int(_tot_time/_dt);
	Eigen::ArrayXXd trajectory = Eigen::ArrayXXd::Zero(3*_particle_radius_step,N);
	double stepdx=_particle_radius_min;
	if(_particle_radius_step>1){
		stepdx = (_particle_radius_max - _particle_radius_min)/double(_particle_radius_step-1);
	}

	_c->set_radius(_chain_radius);
	_c->set_link_length(1.0);
	
	double vol = 0;

	_c->build(_chain_size);
				
	_cg = CollisionGrid(_collision_box_size);
		
	_boundary = VecXd::Zero(6);
	_boundary =_c->get_density_boundary(0.90);
	
	_cg.set_up(_c->get_collision_vec(_boundary) );

	_particle_x_ini = Eigen::Vector3d(floor(_boundary(1)-_boundary(0))+0.5, 
   									  floor(_boundary(3)-_boundary(2))+0.5, 
									  floor(_boundary(5)-_boundary(4))+0.5);

	_particle_v_ini = Eigen::Vector3d(0, 0, 0);
	
	for(int size_idx = 0; size_idx < _particle_radius_step; size_idx++)
	{
		_particle_radius = _particle_radius_min + size_idx*stepdx;
		Particle p = Particle(_dt, _particle_radius, _particle_x_ini, _particle_v_ini, &_cg);
	//	Particle p = Particle(_dt, _particle_radius, _particle_x_ini, _particle_v_ini, NULL);
		p.set_periodic_boundary(_boundary);
		for(int i = 0; i < N; i++)
		{
			p.update();
			trajectory.block(3*size_idx,i,3,1) = p.get_position()-_particle_x_ini.array();
		}
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
		
		_cg.print_box_corners(std::string("../matlab/Distance/debug/grid"));
		
		std::ofstream chain_file;
		chain_file.open("../matlab/Distance/debug/chain", std::fstream::out | std::fstream::trunc);
		if(chain_file.is_open()){
			chain_file << _c->as_array().transpose() << std::endl;
		}else{
			std::cerr << "../matlab/Distance/debug/chain" << std::endl;
		}
		chain_file.close();
		std::cout << "Distance::run_simulation_once:"<<std::endl
			<<"\tPrinted grid to '../matlab/Distance/debug/trajectory'"<<std::endl
			<< "\tTrajectory to '../matlab/Distance/debug/grid'" << std::endl
			<< "\tChain to '../matlab/Distance/debug/chain'" << std::endl;
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

		os <<"Particle Radius Min = " << _particle_radius_min << std::endl;
		os <<"Particle Radius Max = " << _particle_radius_max << std::endl;
		os <<"Particle Radius Steps = " << _particle_radius_step << std::endl;
		
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



void Distance::run_cylinder_test()
{
	std::vector<cg_ptr> test_vec = std::vector<cg_ptr>();

	test_vec.push_back(cg_ptr(new Sphere(Vec3d(0,2,5),1)));
	test_vec.push_back(cg_ptr(new Sphere(Vec3d(2,2,5),1)));
	test_vec.push_back(cg_ptr(new Sphere(Vec3d(2,0,5),1)));
	test_vec.push_back(cg_ptr(new Sphere(Vec3d(2,-2,5),1)));
	test_vec.push_back(cg_ptr(new Sphere(Vec3d(0,-2,5),1)));
	test_vec.push_back(cg_ptr(new Sphere(Vec3d(-2,-2,5),1)));
	test_vec.push_back(cg_ptr(new Sphere(Vec3d(-2,0,5),1)));
	test_vec.push_back(cg_ptr(new Sphere(Vec3d(-2,2,5),1)));


	test_vec.push_back(cg_ptr(new Sphere(Vec3d(0,2,-5),1)));
	test_vec.push_back(cg_ptr(new Sphere(Vec3d(2,2,-5),1)));
	test_vec.push_back(cg_ptr(new Sphere(Vec3d(2,0,-5),1)));
	test_vec.push_back(cg_ptr(new Sphere(Vec3d(2,-2,-5),1)));
	test_vec.push_back(cg_ptr(new Sphere(Vec3d(0,-2,-5),1)));
	test_vec.push_back(cg_ptr(new Sphere(Vec3d(-2,-2,-5),1)));
	test_vec.push_back(cg_ptr(new Sphere(Vec3d(-2,0,-5),1)));
	test_vec.push_back(cg_ptr(new Sphere(Vec3d(-2,2,-5),1)));

	test_vec.push_back(cg_ptr(new Sphere(Vec3d(0,0,-5),2)));
	test_vec.push_back(cg_ptr(new Sphere(Vec3d(0,0,5),2)));
	
	test_vec.push_back(cg_ptr(new Cylinder(1,Vec3d(0,2,-5), Vec3d(0,2,5))));
	test_vec.push_back(cg_ptr(new Cylinder(1,Vec3d(2,2,-5), Vec3d(2,2,5))));
	test_vec.push_back(cg_ptr(new Cylinder(1,Vec3d(2,0,-5), Vec3d(2,0,5))));
	test_vec.push_back(cg_ptr(new Cylinder(1,Vec3d(2,-2,-5), Vec3d(2,-2,5))));
	test_vec.push_back(cg_ptr(new Cylinder(1,Vec3d(0,-2,-5), Vec3d(0,-2,5))));
	test_vec.push_back(cg_ptr(new Cylinder(1,Vec3d(-2,-2,-5), Vec3d(-2,-2,5))));
	test_vec.push_back(cg_ptr(new Cylinder(1,Vec3d(-2,0,-5), Vec3d(-2,0,5))));
	test_vec.push_back(cg_ptr(new Cylinder(1,Vec3d(-2,2,-5), Vec3d(-2,2,5))));

	
	Particle p = Particle(0.01, 0.3,  Vec3d(0,0,0), Vec3d(0,0,0), NULL);
	p.set_test_collision_vector(test_vec);
	int N = int(_tot_time/_dt);
	Eigen::ArrayXXd trajectory = Eigen::ArrayXXd::Zero(3,N);
	for(int i = 0; i<N; i++)
	{
		p.update();
		trajectory.block(0,i,3,1) = p.get_position();
	}

	std::ofstream file;
	file.open("../matlab/Distance/debug/trajectory", 
					std::fstream::out | std::fstream::trunc);
	if(file.is_open()){
		std::cout << "Distance::run_cylinder_test:" << std::endl;
		std::cout << "	Writing to '../matlab/Distance/debug/trajectory'" << std::endl;
		file << trajectory.transpose() << std::endl;
	}else{
		std::cerr << "Failed to open " << std::string("../matlab/Distance/particle_trajectory.dna") << std::endl;
	}
}


bool Distance::Vec3d_equals(Vec3d a, Vec3d b, double tol)
{
	if((std::abs(a(0) - b(0))>tol)  ||
	   (std::abs(a(1) - b(1))>tol)  ||
	   (std::abs(a(2) - b(2))>tol) )
	{
		return false;
	}

	return true;
}
bool Distance::sphere_test_mirror()
{
	// Skapa alla 26 scenarios och testa dom
	bool  failed = false;
	VecXd region = VecXd::Zero(6);
	double x_min = -6;
	double x_max = 4;
	double y_min = -10;
	double y_max = -4;
	double z_min = 4;
	double z_max = 8;
	region << x_min, x_max, y_min, y_max, z_min, z_max;
	double radius = 1;
	
	// order of return is random ish
	// x,y,z
	// 0,y,z
	// x,0,z
	// 0,0,z
	// x,y,0
	// 0,y,0
	// x,0,0

	// Completely inside
	Sphere S1 = Sphere(Vec3d(-3,-6,6), radius);

	// inside but on the edge
	Sphere S2 = Sphere(Vec3d(-5,-9,5), radius);

	// overlapping one edge (z)
	Sphere S3 = Sphere(Vec3d(0,-7,7.5), radius);
	Vec3d S3_pos = Vec3d(0,-7,3.5);


	// overlapping two edges (x+,y+) three returns
	Sphere S4 = Sphere(Vec3d(3.5,-4.5,6), radius);
	Vec3d S41_pos = Vec3d(-6.5,-10.5,6); // x,y
	Vec3d S42_pos = Vec3d(3.5,-10.5,6);  // y
	Vec3d S43_pos = Vec3d(-6.5,-4.5,6);  // x


	// overlapping two edges (x+,z+) three returns
	Sphere S5 = Sphere(Vec3d(3.1,-5.5,7.1), radius);
	Vec3d S51_pos = Vec3d(-6.9,-5.5,3.1); // x,z
	Vec3d S52_pos = Vec3d(3.1,-5.5,3.1);  // z
	Vec3d S53_pos = Vec3d(-6.9,-5.5,7.1);  // x

	// overlapping one corner,  seven returns
	Sphere S6 = Sphere(Vec3d(-5.6,-9.7,7.7), radius);
	Vec3d S61_pos = Vec3d(4.4, -3.7, 3.7);  // x-,y-,z+ // x-,0-,z+
	Vec3d S62_pos = Vec3d(-5.6, -3.7, 3.7); // 0-,y-,z+ // 0-,y-,z+
	Vec3d S63_pos = Vec3d(4.4, -9.7, 3.7);  // x-,0-,z+ // x-,0-,z+
	Vec3d S64_pos = Vec3d(-5.6, -9.7, 3.7); // 0-,0-,z+ // 0-,0-,z+
	Vec3d S65_pos = Vec3d(4.4, -3.7, 7.7);  // x-,y-,0+ // x-,y-,0+
	Vec3d S66_pos = Vec3d(-5.6, -3.7, 7.7); // 0-,y-,0+ // 0-,y-,0+
	Vec3d S67_pos = Vec3d(4.4, -9.7, 7.7);  // x-,0-,0+ // x-,0-,0+

	// overlapping one corner, seven returns
	Sphere S7 = Sphere(Vec3d(3.05,-9.01, 7.1), radius);
	Vec3d S71_pos = Vec3d(-6.95,-3.01, 3.1); // x+,y-,z+
	Vec3d S72_pos = Vec3d(3.05,-3.01, 3.1);  // 0+,y-,z+
	Vec3d S73_pos = Vec3d(-6.95,-9.01, 3.1); // x+,0-,z+
	Vec3d S74_pos = Vec3d(3.05,-9.01, 3.1);  // 0+,0-,z+
	Vec3d S75_pos = Vec3d(-6.95,-3.01, 7.1); // x+,y-,0+
	Vec3d S76_pos = Vec3d(3.05,-3.01, 7.1);  // 0+,y-,0+
	Vec3d S77_pos = Vec3d(-6.95,-9.01, 7.1); // x+,0-,0+

	std::vector<cg_ptr> ret = S1.mirror(region);
	if(ret.size()!= 0)
	{
		failed=true;
	}
	
	ret = S2.mirror(region);
	if(ret.size()!= 0)
	{
		failed=true;
	}
	
	ret = S3.mirror(region);
	if(ret.size()!= 1)
	{
		failed=true;
	}else{
		if(!Vec3d_equals(S3_pos,ret[0]->get_center()))
		{
			failed=true;
		}
	}

	ret = S4.mirror(region);
	if(ret.size() != 3)
	{
		failed = true;
	}else{
		if( !Vec3d_equals(S41_pos, ret[0]->get_center()) ||
			!Vec3d_equals(S42_pos, ret[1]->get_center()) ||
			!Vec3d_equals(S43_pos, ret[2]->get_center()))
		{
			failed = true;
		}
	}
	
	ret = S5.mirror(region);
	if(ret.size() != 3)
	{
		failed = true;
	}else{
		if( !Vec3d_equals(S51_pos, ret[0]->get_center(),0.0000000000001) ||
			!Vec3d_equals(S52_pos, ret[1]->get_center(),0.0000000000001) ||
			!Vec3d_equals(S53_pos, ret[2]->get_center(),0.0000000000001))
		{
			failed = true;
		}
	}
	
	ret = S6.mirror(region);
	if(ret.size() != 7)
	{
		failed = true;
	}else{
		// order of return is random ish
		// 0,0,z
		// x,0,z
		// 0,y,z
		// x,y,z
		// x,0,0
		// 0,y,0
		// x,y,0

		if( !Vec3d_equals(S61_pos, ret[3]->get_center(),0.0000000000001) ||
			!Vec3d_equals(S62_pos, ret[2]->get_center(),0.0000000000001) ||
			!Vec3d_equals(S63_pos, ret[1]->get_center(),0.0000000000001) ||
			!Vec3d_equals(S64_pos, ret[0]->get_center(),0.0000000000001) ||
			!Vec3d_equals(S65_pos, ret[6]->get_center(),0.0000000000001) ||
			!Vec3d_equals(S66_pos, ret[5]->get_center(),0.0000000000001) ||
			!Vec3d_equals(S67_pos, ret[4]->get_center(),0.0000000000001))
		{
			failed = true;
		}
	}
	
	ret = S7.mirror(region);
	if(ret.size() != 7)
	{
		failed = true;
	}else{
		if( !Vec3d_equals(S71_pos, ret[2]->get_center(),0.0000000000001) ||
			!Vec3d_equals(S72_pos, ret[3]->get_center(),0.0000000000001) ||
			!Vec3d_equals(S73_pos, ret[0]->get_center(),0.0000000000001) ||
			!Vec3d_equals(S74_pos, ret[1]->get_center(),0.0000000000001) ||
			!Vec3d_equals(S75_pos, ret[5]->get_center(),0.0000000000001) ||
			!Vec3d_equals(S76_pos, ret[6]->get_center(),0.0000000000001) ||
			!Vec3d_equals(S77_pos, ret[4]->get_center(),0.0000000000001))
		{
			failed = true;
		}
	}


	if(failed)
	{
		std::cerr << "Distance::sphere_test_mirror" << std::endl; 
		std::cerr << "\tMirror-test Failed" << std::endl; 
		return false;	
	}

	return false;
}
