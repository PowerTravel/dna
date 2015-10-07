#include "ChainStatistics.hpp"
#include <iostream>
#include <iomanip>
#include "RChain.hpp"


ChainStatistics::ChainStatistics()
{
	// Simulation parameters
	_nr_strides = 10;
	_stride_len = 50;
	_samples_per_stride = 50;
	_growth_rate = 1.5;
	
	init_plotting_parameters();
	init_arrays();
}

ChainStatistics::~ChainStatistics()
{

}

void ChainStatistics::init_plotting_parameters()
{
	// Plot varibles
	_verbose = true;
	double interval = 1; // How big our update step should be,
						 // 1== update every time
	_increment = 1.f/(_nr_strides*_samples_per_stride);
	_plot_interval = interval * _increment;
	_percent = 0.0;
}

void ChainStatistics::init_arrays()
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


void ChainStatistics::generate()
{
	double theoretical_slope;
	RChain::ChainType _type;
	switch(_type){ 
		case 0:
			theoretical_slope = PHANTOM_SLOPE;
			_type = RChain::ChainType::PHANTOM;
			break;
		case 1:
			theoretical_slope = SAW_SLOPE;
			_type = RChain::ChainType::SAW;
			break;
		case 2:
			theoretical_slope = GLOBULE_SLOPE;
			_type = RChain::ChainType::FG;
			break;
		default:
			theoretical_slope = PHANTOM_SLOPE;
			_type = RChain::ChainType::PHANTOM;
			break;
	}
	

	int N = _stride_len;
	for(int i = 0; i<_nr_strides; i++)
	{
		Eigen::ArrayXd mDist_tmp = Eigen::ArrayXd::Zero(_samples_per_stride);
		for(int j = 0; j<_samples_per_stride; j++)
		{
			RChain c = RChain();
			c.build(N, _type);
			mDist_tmp(j) = c.get_mean_squared_distance();
	
			write_to_terminal(N,i,j);
		}
		nr_links(i) = N;
        mDist(i) = mDist_tmp.sum()/((double) _samples_per_stride);
		mDist_tmp = mDist_tmp - mDist(i);
		mDist_tmp = mDist_tmp*mDist_tmp;
		mDist_var(i) = mDist_tmp.sum() / (N-1);
        mDist_theo(i) = pow(N, theoretical_slope);
		
		N = N * _growth_rate;
	}
	std::cout << std::endl;
	// See if this works and compiles first
	
	mDist_err = ((mDist / mDist_theo).abs()).log();

	write_to_file();
}

void ChainStatistics::write_to_file()
{
	Eigen::MatrixXd result = Eigen::MatrixXd::Zero(_nr_strides, 5);
	result.block(0,0,_nr_strides,1) = nr_links;
	result.block(0,1,_nr_strides,1) = mDist;
	result.block(0,2,_nr_strides,1) = mDist_var;
	result.block(0,3,_nr_strides,1) = mDist_theo;;
	result.block(0,4,_nr_strides,1) = mDist_err;
	std::ofstream file;
	file.open(MEAN_SQUARE_DISTANCE, std::fstream::out | std::fstream::trunc);
	if(file.is_open())
	{
		file << result << std::endl;
	}else{
		std::cerr << "Failed to open " << std::string(MEAN_SQUARE_DISTANCE) << std::endl;
	}
	file.close();
	std::cout <<  "Data saved to " << MEAN_SQUARE_DISTANCE << std::endl;
}

void ChainStatistics::write_to_terminal(int N, int i, int j)
{
	double p = (i*_samples_per_stride + j) * _increment;
	if( (p > _percent) )
	{
		_percent += _plot_interval;
		double disp_percentage = 100*(_percent+_plot_interval);
		std::cout << "\r"<< "["<< std::setw(5)  << std::setprecision(1)<< disp_percentage << std::fixed << "%] "
			<<"Generating " << _samples_per_stride <<" Globules with " << N << " links. " <<  std::flush;
	}
}

