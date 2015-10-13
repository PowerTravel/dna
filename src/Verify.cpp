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
	if( sm.find("NR_STRIDES") != sm.end() )
	{
		_nr_strides  = text_to_int(sm["NR_STRIDES"]);
	}else{
		_valid = false;
	}

	if( sm.find("STRIDE_LEN") != sm.end() )
	{
		_stride_len  = text_to_int(sm["STRIDE_LEN"]);
	}else{
		_valid = false;
	}

	if( sm.find("GROWTH") != sm.end() )
	{
		_growth  = text_to_double(sm["GROWTH"]);
	}else{
		_valid = false;
	}

	if( sm.find("SAMPLES") != sm.end() )
	{
		_samples  = text_to_int(sm["SAMPLES"]);
	}else{
		_valid = false;
	}
	
}

void Verify::init_plotting_parameters()
{
	double interval = 1; // How big our update step should be,
						 // 1== update every time
	_increment = 1.f/(_nr_strides*_samples);
	_plot_interval = interval * _increment;
	_percent = 0.0;
}

void Verify::init_arrays()
{
	// Global
	nr_links    = Eigen::ArrayXd::Zero(_nr_strides);
	
	// Result arrays for mean distance
	mDist   	= Eigen::ArrayXd::Zero(_nr_strides);
	mDist_var	= Eigen::ArrayXd::Zero(_nr_strides);
	mDist_theo  = Eigen::ArrayXd::Zero(_nr_strides);
	mDist_err	= Eigen::ArrayXd::Zero(_nr_strides);

	// Result arrays for mean Radious of gyration
	mRadGyr   	= Eigen::ArrayXd::Zero(_nr_strides);
	mRadGyr_var = Eigen::ArrayXd::Zero(_nr_strides);
	mRadGyr_teo = Eigen::ArrayXd::Zero(_nr_strides);
	mRadGyr_err = Eigen::ArrayXd::Zero(_nr_strides);

	// Result arrays for center of mass
	CM   		= Eigen::ArrayXXd::Zero(_nr_strides,3);
	CM_var   	= Eigen::ArrayXXd::Zero(_nr_strides,3);
	CM_teo  	= Eigen::ArrayXXd::Zero(_nr_strides,3);
	CM_err   	= Eigen::ArrayXXd::Zero(_nr_strides,3);
}
void Verify::print(std::ostream& os)
{
	os <<"Simulation = Verify" << std::endl;
	if(_valid)
	{
		os <<"Out File   = " << _outfile << std::endl;
		os <<"Nr Strides = " << _nr_strides << std::endl;
		os <<"Stride Len = " << _stride_len << std::endl;
		os <<"Growth     = " << _growth << std::endl;
		os <<"Samples    = " << _samples;
	}else{
		os << "Simulation failed to load.";
	}
};

void Verify::set_chain_type()
{
	switch(_chain_type)
	{
		case VAL_BIT_PHANTOM: 
			_theoretical_slope = PHANTOM_SLOPE;
			_t = RChain::ChainType::PHANTOM;
			break;

		case VAL_BIT_SAW:
			_theoretical_slope = SAW_SLOPE;
			_t = RChain::ChainType::SAW;
			break;

		case VAL_BIT_FG:
			_theoretical_slope = GLOBULE_SLOPE;
			_t = RChain::ChainType::FG;
			break;
	}
}

void Verify::apply()
{
	print_pre_info();

	int max_idx = _nr_strides;
	int measures = _samples;

	nr_links(0) = _stride_len;
	for(int i = 1; i<max_idx; i++)
	{
		nr_links(i) = nr_links(i-1)*_growth;
	}
	int chain_length = nr_links(max_idx-1);

	// Rows contain different measures
	// columns contain different lengths,
	Eigen::ArrayXXd mDist_tmp = Eigen::ArrayXXd::Zero(measures, max_idx);
	Eigen::ArrayXXd mRadGyr_tmp = Eigen::ArrayXXd::Zero(measures, max_idx);
	Eigen::ArrayXXd CM_tmp = Eigen::ArrayXXd::Zero(measures,3 * max_idx);
	for(int i = 0; i<measures; i++)
	{
		RChain c = RChain();
		c.build(chain_length, _t);

		// Gather mean distance data
		for(int j = 0; j<max_idx; j++)
		{
			// Mean distance
			mDist_tmp(i,j) += c.get_mean_squared_distance(0, nr_links(j));

			// Radious of gyration
			mRadGyr_tmp(i,j) += c.get_rad_of_gyr(0, nr_links(j));
			
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

		/*  Mean square dispacement  */
		// Calculate the mean and variance
		tmp_arr = mDist_tmp.col(i);
		mv = get_mean_and_variance(tmp_arr);
		mDist(i) = mv(0);
		mDist_var(i) = mv(1);
		// Generate the theoretical slope
        mDist_theo(i) = pow(nr_links(i), _theoretical_slope);
		
		/* Center of mass */
		for(int j = 0; j<3; j++)
		{
			mv = get_mean_and_variance(CM_tmp.col(3*i+j));
			CM(i,j) = mv(0);
			CM_var(i,j) = mv(1);
		}

		/* Radius of gyration */
		// THIS IS GIVING WEIRD RESULTS I THINK	
		mv = get_mean_and_variance(mRadGyr_tmp.col(i));
		mRadGyr(i) = mv(0);  
		mRadGyr_var(i) = mv(1);
	}
	std::cout << mRadGyr << std::endl << std::endl;
	std::cout << mRadGyr_var << std::endl << std::endl;
	// The log of the error between the theoretical and the measured distance	
	mDist_err = ((mDist / mDist_theo).abs()).log();

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

/*
void Verify::apply()
{
	print_pre_info();
	int N = _stride_len;
	for(int i = 0; i<_nr_strides; i++)
	{
		Eigen::ArrayXd mDist_tmp = Eigen::ArrayXd::Zero(_samples);
		for(int j = 0; j<_samples; j++)
		{
			RChain c = RChain();
			c.build(N, _t);
			mDist_tmp(j) = c.get_mean_squared_distance();
			
			if(verbose)
			{
				write_to_terminal(N,i,j);
			}
		}
		nr_links(i) = N;
        mDist(i) = mDist_tmp.sum()/((double) _samples);
		mDist_tmp = mDist_tmp - mDist(i);
		mDist_tmp = mDist_tmp*mDist_tmp;
		mDist_var(i) = mDist_tmp.sum() / (N-1);
        mDist_theo(i) = pow(N, _theoretical_slope);
		
		N = N * _growth;
	}
	if(verbose)
	{
		std::cout << std::endl;
	}
	
	mDist_err = ((mDist / mDist_theo).abs()).log();

	write_to_file();
	
	print_post_info();
}
*/
void Verify::write_to_file()
{
	Eigen::MatrixXd result = Eigen::MatrixXd::Zero(_nr_strides, 5);
	result.block(0,0,_nr_strides,1) = nr_links;
	result.block(0,1,_nr_strides,1) = mDist;
	result.block(0,2,_nr_strides,1) = mDist_var;
	result.block(0,3,_nr_strides,1) = mDist_theo;;
	result.block(0,4,_nr_strides,1) = mDist_err;
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
