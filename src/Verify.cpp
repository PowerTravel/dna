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
	nr_links    = Eigen::VectorXd::Zero(_nr_strides);
	
	// Result arrays for mean distance
	mDist   	= Eigen::VectorXd::Zero(_nr_strides);
	mDist_var	= Eigen::VectorXd::Zero(_nr_strides);
	mDist_theo  = Eigen::VectorXd::Zero(_nr_strides);
	mDist_err	= Eigen::VectorXd::Zero(_nr_strides);

	// Result arrays for mean Radious of gyration
	mRadGyr   	= Eigen::VectorXd::Zero(_nr_strides);
	mRadGyr_var = Eigen::VectorXd::Zero(_nr_strides);
	mRadGyr_teo = Eigen::VectorXd::Zero(_nr_strides);
	mRadGyr_err = Eigen::VectorXd::Zero(_nr_strides);

	// Result arrays for center of mass
	CM   		= Eigen::VectorXd::Zero(_nr_strides);
	CM_var   	= Eigen::VectorXd::Zero(_nr_strides);
	CM_teo  	= Eigen::VectorXd::Zero(_nr_strides);
	CM_err   	= Eigen::VectorXd::Zero(_nr_strides);
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
			<<"Generating " << _samples <<" Globules with " << N << " links. " <<  std::flush;
	}
}
