#include "Verify.hpp"
#include <iostream>
#include <iomanip>
Verify::Verify()
{

}

Verify::Verify(std::map<std::string, std::string> sm) : Simulation(sm)
{
	_simulation_type = val_map.at("verify");
	init_simulaition_parameters(sm);
	set_chain_type();

	init_plotting_parameters();
	
	init_arrays();
}
Verify::~Verify()
{

}

void Verify::init_simulaition_parameters(std::map<std::string, std::string> sm)
{
	if( sm.find("SIZE") != sm.end() )
	{
		_size  = text_to_int(sm["SIZE"]);
	}else{
		_valid = false;
	}

	if( sm.find("STRIDES") != sm.end() )
	{
		_strides  = text_to_int(sm["STRIDES"]);
	}else{
		_valid = false;
	}

	if( sm.find("EXP") != sm.end() )
	{
		_exp  = text_to_bool(sm["EXP"]);
	}else{
		_valid = false;
	}

	if( sm.find("SAMPLES") != sm.end() )
	{
		_samples  = text_to_int(sm["SAMPLES"]);
	}else{
		_valid = false;
	}
	
	if(_valid){
	
		double N =(double) _size;
		double s =(double) _strides;

		if( N/s < 1 )
		{
			_valid = false;
		}else{

			if(_exp)
			{
				set_exponential_links();
			}else{
				set_linear_links();
			}
		}
	}
}

void Verify::set_chain_type()
{
	switch(_chain_type)
	{
		case VAL_BIT_PHANTOM: 
			_theoretical_slope = PHANTOM_SLOPE;
			_theoretical_Rg_slope = PHANTOM_SLOPE;
			_t = Chain::ChainType::PHANTOM;
			break;

		case VAL_BIT_SAW:
			_theoretical_slope = SAW_SLOPE;
			_theoretical_Rg_slope = SAW_SLOPE;
			_t = Chain::ChainType::SAW;
			break;

		case VAL_BIT_FG:
			_theoretical_slope = GLOBULE_SLOPE;
			_theoretical_Rg_slope = GLOBULE_SLOPE;
			_t = Chain::ChainType::FG;
			break;
	}
}

void Verify::set_exponential_links()
{
	double N =(double) _size;
	double s =(double) _strides;

//	double start_link = 2;
//	double start_link = N/100;
	double start_link = 10;

	if(start_link<2){
		start_link = 2;
	}

	double k = log(N /start_link ) / (s-1);

	nr_links = Eigen::ArrayXd::Zero(s);
	for(int i = 0; i< s; i++)
	{
		nr_links(i) = floor( start_link * exp(k * i) );
	}
}

void Verify::set_linear_links()
{
	double N =(double) _size;
	double s =(double) _strides;
	_step_size = N /s;
	if(_step_size < 2)
	{
		_start_link = 2;
		_strides = _strides - 1;
	}else{
		_start_link = floor(_step_size);	
	}

	nr_links = Eigen::ArrayXd::Zero(_strides);
	for(int i = 0; i<_strides; i++)
	{
		nr_links(i) =floor( i * _step_size ) + _start_link;
	}
}

void Verify::init_plotting_parameters()
{
	double interval = 1; // How big our update step should be,
						 // 1== update every time
	_increment = 1.f/(_strides*_samples);
	_plot_interval = interval * _increment;
	_percent = 0.0;
}
void Verify::init_arrays()
{
	// Result arrays for mean distance
	mDist   	= Eigen::ArrayXd::Zero(_strides);
	mDist_var	= Eigen::ArrayXd::Zero(_strides);
	mDist_theo  = Eigen::ArrayXd::Zero(_strides);
	mDist_err	= Eigen::ArrayXd::Zero(_strides);

	// Result arrays for mean Radious of gyration
	mRadGyr   	= Eigen::ArrayXd::Zero(_strides);
	mRadGyr_var = Eigen::ArrayXd::Zero(_strides);
	mRadGyr_theo= Eigen::ArrayXd::Zero(_strides);
	mRadGyr_err = Eigen::ArrayXd::Zero(_strides);

	// Result arrays for center of mass
	CM   		= Eigen::ArrayXXd::Zero(_strides,3);
	CM_var   	= Eigen::ArrayXXd::Zero(_strides,3);
	CM_theo 	= Eigen::ArrayXXd::Zero(_strides,3);
	CM_err   	= Eigen::ArrayXXd::Zero(_strides,3);
	
}

void Verify::print(std::ostream& os)
{
	os <<"Simulation = Verify" << std::endl;
	if(_valid)
	{
		os <<"Out File   = " << _outfile << std::endl;
		os <<"Chain Size = " << _size<< std::endl;
		os <<"Nr Strides = " << _strides << std::endl;
		os <<"Samples    = " << _samples;
		os <<"Exponential= " << _exp << std::endl;
	}else{
		os << "Simulation failed to load.";
	}
};

void Verify::apply()
{
	print_pre_info();

	int max_idx = nr_links.size();
	int measures = _samples;

	int chain_length = nr_links(max_idx-1);

	// Theoretical Values
	CM_theo 	= Eigen::ArrayXXd::Zero(max_idx,3);
	mDist_theo  = nr_links.pow(_theoretical_slope);
	mRadGyr_theo= nr_links.pow(_theoretical_Rg_slope);

	// Rows contain different measures
	// columns contain different lengths,
	Eigen::ArrayXXd mDist_tmp = Eigen::ArrayXXd::Zero(measures, max_idx);
	Eigen::ArrayXXd mRadGyr_tmp = Eigen::ArrayXXd::Zero(measures, max_idx);
	Eigen::ArrayXXd CM_tmp = Eigen::ArrayXXd::Zero(measures,3 * max_idx);
	for(int i = 0; i<measures; i++)
	{
		Chain c = Chain();
		c.build(chain_length, _t);

		// Gather mean distance data
		for(int j = 0; j<max_idx; j++)
		{
			// Mean distance
			mDist_tmp(i,j) = c.get_mean_squared_distance(0, nr_links(j));

			// Radious of gyration
			mRadGyr_tmp(i,j) = c.get_rad_of_gyr(0, nr_links(j));
			
			// Center of mass
			Eigen::Array3d  tmp_cm = c.get_CM(0, nr_links(j));
			CM_tmp.block(i,3*j,1,3) =CM_tmp.block(i,3*j,1,3) + tmp_cm.transpose();
			if(verbose)
			{
				write_to_terminal(chain_length,i,j);
			}
		}
	}
	// Calculate mean distance
	for(int i = 0; i<max_idx; i++)
	{

		Eigen::ArrayXd tmp_arr = Eigen::ArrayXd::Zero(measures);
		Eigen::Vector2d mv = Eigen::Vector2d::Zero();

		// Mean square dispacement 
		// Calculate the mean and variance
		tmp_arr = mDist_tmp.col(i);
		mv = get_mean_and_variance(tmp_arr);
		mDist(i) = mv(0);
		mDist_var(i) = mv(1);
		
		// Center of mass 
		for(int j = 0; j<3; j++)
		{
			mv = get_mean_and_variance(CM_tmp.col(3*i+j));
			CM(i,j) = mv(0);
			CM_var(i,j) = mv(1);
		}

		// Radius of gyration 
		mv = get_mean_and_variance(mRadGyr_tmp.col(i));
		mRadGyr(i) = mv(0);
		mRadGyr_var(i) = mv(1);
	}
	
	// The log of the error between the theoretical and the measured distance	
	// This can only be caluclated where I know the theoretical values
	mDist_err = ((mDist - mDist_theo)/mDist_theo).abs();

	CM_err = (CM - CM_theo)/(1+CM_theo).abs();

	mRadGyr_err = (mRadGyr - mRadGyr_theo).abs()/mRadGyr_theo;

	write_to_file();

	print_post_info();
}

Eigen::Vector2d Verify::get_mean_and_variance(Eigen::ArrayXd in_data )
{
	Eigen::Vector2d ret = Eigen::Vector2d::Zero();
	// Mean
	ret(0) = in_data.sum()/((double) in_data.size());
	ret(1) = (in_data-ret(0)).pow(2).sum();
	
	return ret;
}
void Verify::write_to_file()
{
	int idx = nr_links.size();
	Eigen::MatrixXd result = Eigen::MatrixXd::Zero(idx, 21);
	result.block(0,0,idx,1) = nr_links;
	result.block(0,1,idx,1) = mDist;
	result.block(0,2,idx,1) = mDist_var;
	result.block(0,3,idx,1) = mDist_theo;
	result.block(0,4,idx,1) = mDist_err;

	result.block(0,5,idx,1) = mRadGyr;
	result.block(0,6,idx,1) = mRadGyr_var;
	result.block(0,7,idx,1) = mRadGyr_theo;
	result.block(0,8,idx,1) = mRadGyr_err;

	result.block(0,9, idx,3)  = CM;
	result.block(0,12,idx,3) = CM_var;
	result.block(0,15,idx,3) = CM_theo;
	result.block(0,18,idx,3) = CM_err;

	std::ofstream file;
	file.open(_outfile, std::fstream::out | std::fstream::trunc);
	if(file.is_open())
	{
		file << result << std::endl;
	}else{
		std::cerr << "Failed to open " << std::string(_outfile) << std::endl;
	}
	file.close();
}

void Verify::write_to_terminal(int N, int i, int j)
{
	double p = (i*_samples + j) * _increment;
	if( (p > _percent) )
	{
		_percent += _plot_interval;
		double disp_percentage = 100*(_percent+_plot_interval);
		std::cout << "\r"<< "["<< std::setw(5)  << std::setprecision(1)<< disp_percentage << std::fixed << "%] "
			<<"Generating " << _samples <<" " <<dictionary.at(_chain_type) <<"s with " << N << " links. " <<  std::flush;
	}
}
