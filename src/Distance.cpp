#include "Distance.hpp"
#include "Statistics.hpp"

Distance::Distance(std::map<std::string, std::string> sm) : Simulation(sm)
{
	_simulation_type = val_map.at("distance");
	init_simulaition_parameters(sm);
}

void Distance::init_simulaition_parameters(std::map<std::string, std::string> sm)
{
	if( sm.find("SIZE") != sm.end() )
	{
		_size  = text_to_int(sm["SIZE"]);
	}else{
		_valid = false;
	}

	if( sm.find("BOX_SIZE") != sm.end() )
	{
		_box_size  = text_to_double(sm["BOX_SIZE"]);
	}else{
		_valid = false;
	}

	if( sm.find("RADIUS") != sm.end() )
	{
		_rad  = text_to_double(sm["RADIUS"]);
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
	print_pre_info();
	if(!_valid)
	{
		std::cerr << "Verify not valid. Exiting" << std::endl;
		return;
	}
	std::cout << this << std::endl;

	// Build chain and setup collision grid
	_c->set_radius(_rad);
	_c->set_link_length(1.0);
	_c->build(_size);
	CollisionGrid cg = CollisionGrid(_box_size);

	int  max_axis = get_max(_c->axis_length());
	cg.set_up(_c->get_collision_vec() , max_axis );
	cg.print_box_corners(std::string("../matlab/Distance/grid.txt"));



	double radius = 0.2;
	Eigen::Vector3d x_ini = Eigen::Vector3d(0.5, 0.5, 0.5);
	Eigen::Vector3d v_ini = Eigen::Vector3d(0, 0, 0);
	double tot_time = 10;
	double dt = 0.01;
	
	int Nr_simulations = 1000;

	Eigen::ArrayXd distance = Eigen::ArrayXd::Zero(Nr_simulations);
	Eigen::ArrayXXd final_pos_arr = Eigen::ArrayXXd::Zero(3,Nr_simulations);
	
	for(int i = 0; i < Nr_simulations; i++)
	{
		Eigen::ArrayXXd trajectory = run_simulation_once(radius, x_ini, v_ini, tot_time, dt, &cg);
		// Nr of columns
		int N = trajectory.outerSize();
		Eigen::Vector3d final_pos_tmp = trajectory.block(0,N-1, 3, 1);
		final_pos_arr.block(0,i,3,1) = final_pos_tmp;
		distance(i) = final_pos_tmp.norm();
	}
	Eigen::Vector3d final_average_pos;
	Eigen::Vector3d final_average_pos_var;

	// X
	Eigen::VectorXd data_1d = final_pos_arr.block(0,0,1,Nr_simulations).transpose();
	Eigen::Vector2d	tmp2d = Statistics::get_mean_and_variance(data_1d);
	final_average_pos(0) = tmp2d(0);
	final_average_pos_var(0) = tmp2d(1);

	// Y
	data_1d = final_pos_arr.block(1,0,1,Nr_simulations).transpose();
	tmp2d = Statistics::get_mean_and_variance(data_1d);
	final_average_pos(1) = tmp2d(0);
	final_average_pos_var(1) = tmp2d(1);

	// Z
	data_1d = final_pos_arr.block(2,0,1,Nr_simulations).transpose();
	tmp2d = Statistics::get_mean_and_variance(data_1d);
	final_average_pos(2) = tmp2d(0);
	final_average_pos_var(2) = tmp2d(1);


	Eigen::Vector2d meanVar = Statistics::get_mean_and_variance(distance);
	std::cout << meanVar.transpose() << std::endl;
	std::cout << final_average_pos.transpose() << std::endl;
	std::cout << final_average_pos_var.transpose() << std::endl;

/*
	std::ofstream file;
	file.open(_outfile, std::fstream::out | std::fstream::trunc);
	if(file.is_open()){
		file << p << std::endl;
	}else{
		std::cerr << "Failed to open " << std::string(_outfile) << std::endl;
	}

	file.close();
*/
	print_post_info();
}


Eigen::ArrayXXd Distance::run_simulation_once(double radius, 
							Eigen::Vector3d x_ini, Eigen::Vector3d v_ini, 
							double tot_time, double dt, 
							CollisionGrid* cg)
{
	Particle p = Particle(radius, x_ini, v_ini, cg);
	int N = int(tot_time/dt);
	Eigen::ArrayXXd trajectory = Eigen::ArrayXXd::Zero(3,N);
	for(int i = 0; i < N; i++)
	{
		p.update(dt, Eigen::Array3d(0,0,0) );
		trajectory.block(0,i,3,1) = p.get_position();
	}

	return trajectory;
}

void Distance::print(std::ostream& os)
{
	os <<"Simulation = Distance" << std::endl;
	if(_valid)
	{
		os <<"Out File   = " << _outfile << std::endl;
		os <<"Chain Size = " << _size<< std::endl;
		os <<"Use Weight = ";
		if(_weight)
		{
			os << "true" << std::endl;
		}else{
			os << "false" << std::endl;
		}
		os <<"Allow selfintersection = ";
		if(_selfint)
		{
			os << "true" << std::endl;
		}else{
			os << "false" << std::endl;
		}
		os <<"Box_Size of collision grid = " << _box_size << std::endl;
		os <<"Radius of chain links  = " << _rad << std::endl;
	}else{
		os << "Simulation failed to load.";
	}
};
