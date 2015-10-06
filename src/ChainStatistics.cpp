#include "ChainStatistics.hpp"

#include "RChain.hpp"


ChainStatistics::ChainStatistics()
{
	// Simulation parameters
	_nr_strides = 40;
	_stride_len = 10;
	_samples_per_stride = 30;
	_growth_rate = 1.1;
	
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
	_increment = 1.f/(nr_strides*samples_per_stride);
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
	RChain::ChainType type;
	switch(_type){ 
		case 0:
			theoretical_slope = PHANTOM_SLOPE;
			type = RChain::ChainType::PHANTOM;
			break;
		case 1:
			theoretical_slope = SAW_SLOPE;
			type = RChain::ChainType::SAW;
			break;
		case 2:
			theoretical_slope = GLOBULE_SLOPE;
			type = RChain::ChainType::GLOBULE;
			break;
		default:
			theoretical_slope = PHANTOM_SLOPE;
			type = RChain::ChainType::PHANTOM;
			break;
	}
	

	int N = _stride_len;
	for(int i = 0; i<_nr_strides; i++)
	{
		Eigen::ArrayXd distance_tmp = Eigen::ArrayXd::Zero(samples_per_stride);
		for(int j = 0; j<_samples_per_stride; j++)
		{
			RChain c = RChain();
			c.build(N, type);
			distance_tmp(j) = c.get_mean_squared_distance();
	
			write_to_terminal(N);
		}
		nr_links(i) = N;
        mean_distance(i) = distance_tmp.sum()/((double) samples_per_stride);
		distance_tmp = distance_tmp - mean_distance(i);
		distance_tmp = distance_tmp*distance_tmp;
		mean_distance_var(i) = distance_tmp.sum() / (N-1);
        theoretical(i) = pow(N, theoretical_slope);
		
		N = N * _growt_rate;
	}
	std::cout << std::endl;
	// See if this works and compiles first
/*
	log_slope_error = ((mean_distance / theoretical).abs()).log();
	
	// Writing data to file 
	// ****************************
	Eigen::MatrixXd result = Eigen::MatrixXd::Zero(nr_strides, 5);
	result.block(0,0,nr_strides,1) = nr_links;
	result.block(0,1,nr_strides,1) = mean_distance;
	result.block(0,2,nr_strides,1) = mean_distance_var;
	result.block(0,3,nr_strides,1) = theoretical;
	result.block(0,4,nr_strides,1) = log_slope_error;
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
	// *****************************
	*/
}

void ChainStatistics::write_to_terminal(int N)
{
	double p = (i*samples_per_stride + j) * increment;
	if( (p > _percent) )
	{
		_percent += _plot_interval;
		double disp_percentage = 100*(_percent+_plot_interval);
		std::cout << "\r"<< "["<< std::setw(5)  << std::setprecision(1)<< disp_percentage << std::fixed << "%] "
			<<"Generating " << _samples_per_stride <<" Globules with " << N << " links. " <<  std::flush;
	}
}

