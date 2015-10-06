#include <iostream>
#include <Eigen/Dense>
#include <random>
#include <cmath>
#include <iomanip> // setw()
#include "Chain.hpp"
#include "RChain.hpp"

#ifndef OUT_FILES
#define INITIAL_STATE "../data/initial_state.m"
#define ANIMATION = "../data/animation.m"
#define MEAN_SQUARE_DISTANCE "../data/meanSquareDistance.m"
#endif // OUT_FILES

#ifndef THEORETICAL_SLOPES
#define PHANTOM_SLOPE 1/2.f
#define SAW_SLOPE 3/(2.f+3.f)
#define GLOBULE_SLOPE 1/3.f
#endif // THEORETICAL_SLOPES

/*
 * 	Program dna
 *	Generates a fractal globule and prints data to a file
 */


void generate_end_to_end_plot();
void build_initial_state(int n);
void get_average_radious_of_gyration(int N);



int main(int argc, char* argv[])
{
	if(argc == 3)
	{
		int N = atoi(argv[2]);
		if(argv[1][0] == 'i')
		{
			build_initial_state(N);
		}else if( argv[1][0] == 'r'){
			get_average_radious_of_gyration(N);
		}
	}else{
		generate_end_to_end_plot();
	}


	return 0;
}

void build_initial_state(int n)
{
	RChain c = RChain();
	c.build(n,RChain::ChainType::SAW);
	// Print to file
	std::ofstream file;
	file.open(INITIAL_STATE, std::fstream::out | std::fstream::trunc);
	if(file.is_open())
	{
		file << c << std::endl;
	}
	file.close();
}

void get_average_radious_of_gyration(int N)
{
	int tests = 100;

	double mean = 0;
	Eigen::ArrayXd arr = Eigen::ArrayXd::Zero(tests);
	double var = 0;

	for(int i=0; i<tests; i++)
	{
		RChain c = RChain();
		c.build(N , RChain::ChainType::PHANTOM);
		arr(i) =  c.get_rad_of_gyr();
	}
	mean = arr.sum()/((double) tests);	
	
	for(int i=0; i<tests; i++)
	{
		double ax =(mean - arr(i));
		var += ax*ax; 
	}
	var = var/((double)N);
	std::cout << "Rad = " << mean << "  stdavvikelse = " << sqrt(var) << std::endl; 
}

// linear
void generate_end_to_end_plot()
{

	// PARAMETERS
	int nr_strides = 40;
	int stride_len = 10;
	double stride_growth = 1.1;
	int samples_per_stride = 100;

	double interval = 1;
	double theoretical_slope = SAW_SLOPE;
	RChain::ChainType type = RChain::ChainType::SAW;
	

	// Result arrays for mean distance
	Eigen::ArrayXd nr_links        		= Eigen::VectorXd::Zero(nr_strides);
	Eigen::ArrayXd mean_distance   		= Eigen::VectorXd::Zero(nr_strides);
	Eigen::ArrayXd mean_distance_var    = Eigen::VectorXd::Zero(nr_strides);
	Eigen::ArrayXd theoretical     		= Eigen::VectorXd::Zero(nr_strides);
	Eigen::ArrayXd log_slope_error 		= Eigen::VectorXd::Zero(nr_strides);

	// Result arrays for mean Radious of gyration
	Eigen::ArrayXd rad_gyration    		= Eigen::VectorXd::Zero(nr_strides);
	Eigen::ArrayXd rad_gyration_var   	= Eigen::VectorXd::Zero(nr_strides);

	double increment = 1.f/(nr_strides*samples_per_stride);
	const double plot_interval = interval * increment;
	double percent = 0.0;
	for(int i = 0; i<nr_strides; i++)
	{
		int N =stride_len;
		Eigen::ArrayXd distance_tmp = Eigen::ArrayXd::Zero(samples_per_stride);
		for(int j = 0; j<samples_per_stride; j++)
		{
			RChain c = RChain();
			c.build(N, type);
			distance_tmp(j) = c.get_mean_squared_distance();

		// writing progress to terminal
		// ****************************
			double p = (i*samples_per_stride + j) * increment;
			if( (p > percent) )
			{
				percent += plot_interval;
				double disp_percentage = 100*(percent+plot_interval);
				std::cout << "\r"<< "["<< std::setw(5)  << std::setprecision(1)<< disp_percentage << std::fixed << "%] "
					<<"Generating " << samples_per_stride <<" Globules with " << N << " links. " <<  std::flush;
			}
		// ****************************
		}

		stride_len = stride_len * stride_growth;
		nr_links(i) = N;
        mean_distance(i) = distance_tmp.sum()/((double) samples_per_stride);
		distance_tmp = distance_tmp - mean_distance(i);
		distance_tmp = distance_tmp*distance_tmp;
		mean_distance_var(i) = distance_tmp.sum() / (N-1);
        theoretical(i) = pow(N, theoretical_slope);
	}
	std::cout << std::endl;

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
}

