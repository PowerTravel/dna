#include "Distance.hpp"
#include "Statistics.hpp"
#include <iomanip>

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

	_c->set_radius(_rad);
	_c->set_link_length(1.0);
	_c->build(_size);
	CollisionGrid cg = CollisionGrid(_box_size);

	int  max_axis = get_max(_c->axis_length());
	cg.set_up(_c->get_collision_vec() , max_axis );
	cg.print_box_corners(std::string("../matlab/Distance/grid.txt"));


	// TODO: Add some of these to the config file
	double radius = 0.2;
	Eigen::Vector3d x_ini = Eigen::Vector3d(0.5, 0.5, 0.5);
	Eigen::Vector3d v_ini = Eigen::Vector3d(0, 0, 0);
	double tot_time = 100;
	double dt = 0.01;
	int T = int(tot_time /dt);

	
	nr_simulations = 1000;
	nr_data_points = 50.0;
	int start_point = 100;


	time_steps = Statistics::make_exponential_points_array(T, nr_data_points, start_point );
	time_step_mean = (time_steps.segment(0, nr_data_points) + time_steps.segment(1, nr_data_points)) / 2;

	Eigen::ArrayXXd binned_distance_data = 
							Eigen::ArrayXXd::Zero(nr_data_points, 3*nr_simulations);


	for(int i = 0; i < nr_simulations; i++)
	{
		Eigen::ArrayXXd trajectory = run_simulation_once(radius, x_ini, v_ini, tot_time, dt, &cg);

		for(int j = 0; j<nr_data_points; j++)
		{
			int start = floor(time_steps(j));
			int len = floor(time_steps(j+1) - start);
			if(len == 0)
			{
				len = 1;
			}

			Eigen::ArrayXXd tmp_mat = 
				trajectory.block(0, start, 3, len);
			tmp_mat.row(0) = tmp_mat.row(0) - x_ini(0);
			tmp_mat.row(1) = tmp_mat.row(1) - x_ini(1);
			tmp_mat.row(2) = tmp_mat.row(2) - x_ini(2);

			binned_distance_data(j,3*i+0) = tmp_mat.row(0).mean();
			binned_distance_data(j,3*i+1) = tmp_mat.row(1).mean();	
			binned_distance_data(j,3*i+2) = tmp_mat.row(2).mean();


		}
		write_to_terminal(i,nr_data_points);
	}

	D = Eigen::ArrayXd::Zero(nr_data_points);
	D_var = Eigen::ArrayXd::Zero(nr_data_points);
	Eigen::ArrayXXd D_tmp = Eigen::ArrayXXd::Zero(nr_data_points, nr_simulations);
	

	P = Eigen::ArrayXXd::Zero(nr_data_points, 3);
	P_var = Eigen::ArrayXXd::Zero(nr_data_points, 3);
	Eigen::ArrayXXd P_tmp_x = Eigen::ArrayXXd::Zero(nr_data_points, nr_simulations);
	Eigen::ArrayXXd P_tmp_y = Eigen::ArrayXXd::Zero(nr_data_points, nr_simulations);
	Eigen::ArrayXXd P_tmp_z = Eigen::ArrayXXd::Zero(nr_data_points, nr_simulations);

	for(int i = 0; i < nr_data_points; i++)
	{
		for(int j = 0; j < nr_simulations; j++)
		{
			Eigen::Vector3d pos  = binned_distance_data.block(i,3*j,1,3).transpose();
	
			P_tmp_x(i,j) = pos(0);
			P_tmp_y(i,j) = pos(1);
			P_tmp_z(i,j) = pos(2);

			D_tmp(i,j) = pos.norm();
		}
	}
	

	for(int i = 0; i<nr_data_points; i++)
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

	print_post_info();
	
}

void Distance::write_to_terminal(int i, int j)
{
	double p = ( ( double) i*time_steps(time_steps.size()-1) + j) / ((double) nr_simulations*time_steps(time_steps.size()-1)) * 100.f;
	std::cout << "\r"<< "["<< std::setw(5)  << std::setprecision(1)<< p << std::fixed << "%] "
			<<"Running..." << std::flush;
}
void Distance::write_to_file()
{
	int idx = time_steps.size()-1;
	Eigen::MatrixXd result = Eigen::MatrixXd::Zero(idx, 9);

	result.block(0,0,idx,1) = time_step_mean;
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
		os <<"Box_Size of collision grid = " << _box_size << std::endl;
		os <<"Radius of chain links  = " << _rad << std::endl;
	}else{
		os << "Simulation failed to load.";
	}
};
