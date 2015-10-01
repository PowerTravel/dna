#include <iostream>
#include <Eigen/Dense>
#include <random>
#include <cmath>
#include <iomanip> // setw()
#include "Chain.hpp"

#ifndef OUT_FILES
#define INITIAL_STATE "../data/initial_state.m"
#define ANIMATION = "../data/animation.m"
#define MEAN_SQUARE_DISTANCE "../data/meanSquareDistance.m"
#endif // OUT_FILES


void generate_end_to_end_plot();
void build_initial_state(int n);

double plotPercentage(double i, double N, double percent);

int main(int argc, char* argv[])
{
	int N = 1000000;
	if(argc >= 2)
	{
		N = atoi(argv[1]);
	}
	
	//build_initial_state(N);


	generate_end_to_end_plot();


	return 0;
}

void build_initial_state(int n)
{
	Chain c = Chain();
	c.generateGlobule(n);
	//c.print_knots();

	// Print to file
	std::ofstream file;
	file.open(INITIAL_STATE, std::fstream::out | std::fstream::trunc);
	if(file.is_open())
	{
		file << c << std::endl;
	}
	file.close();
}

// linear
void generate_end_to_end_plot()
{
	int nr_strides = 7;
	int stride_len = 10;
	double stride_growth = 2;
	int samples_per_stride = 100;

	Eigen::ArrayXd tmp = Eigen::ArrayXd::Zero(samples_per_stride);
	Eigen::MatrixXd result = Eigen::MatrixXd::Zero(nr_strides, 4); // First column is the number of links used;
																	// Second column is the measured result, 
																	// Third column is the variance
																	// Fourth column is theoretical result
	double percent = 0.0;
	double plot_interval = 0.0001;

	for(int i = 0; i<nr_strides; i++)
	{
		int N =floor((i+1)*stride_len);
		for(int j = 0; j<samples_per_stride; j++)
		{
			Chain c = Chain();
			c.generateGlobule(N);
			tmp(j) = c.get_mean_squared_distance();
		
		
			// writing progress to terminal
			double p = ((double) (i*samples_per_stride + j))/((double) nr_strides*samples_per_stride);
			//std::cout << p <<" = " << i << " * " << nr_strides << " + " << j << " /( " << nr_strides << " * " << samples_per_stride << " )" << std::endl;

			if(p >= percent )
			{
				percent += plot_interval;
				double disp_percentage = (100.f/nr_strides) * percent*100.f;
				std::cout << "\r"<< "["<< std::setw(6)  << std::setprecision(2)<< disp_percentage << std::fixed << "%] "
					<<"Generating " << samples_per_stride <<" Globules with " << N << " links. Nr of retries = " << std::setw(3) <<c._redo << std::flush;
			}
		}
		stride_len = stride_len * stride_growth;

		// Nr of links
		result(i,0) = N;
		// Average
		result(i,1) = tmp.sum()/((double) samples_per_stride);
		// Variance
		tmp = tmp - result(i,1);
		tmp = tmp*tmp;
		result(i,2) = tmp.sum() / (N-1);

		// Theoretical
		result(i,3) = pow(N,1/3.0f);
	}
	std::cout <<"Done!" << std::endl;


	// Writing to file
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

