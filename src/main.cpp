#include <iostream>
#include <Eigen/Dense>
#include <random>
#include <cmath>
#include "Chain.hpp"

#ifndef OUT_FILES
#define INITIAL_STATE "../data/initial_state.m"
#define ANIMATION = "../data/animation.m"
#define MEAN_SQUARE_DISTANCE "../data/meanSquareDistance.m"
#endif // OUT_FILES


void print_chain_to_file(Chain& c);
void generate_n_samples(int n);

int main(int argc, char* argv[])
{
	int N = 10;
	if(argc >= 2)
	{
		N = atoi(argv[1]);
	}
	// Generate a chain or load chain configuration from file 
	
	//Simulate the chain 
	/*
	Chain c = Chain();
	c.generateGlobule(N);
	print_chain_to_file(c);
	//c.print_knots();
	*/

	generate_n_samples(100);

	 //Print data to file 
	
	return 0;
}


void generate_n_samples(int n)
{
	Chain c = Chain();
	Eigen::VectorXd res  = Eigen::VectorXd::Zero(n);
	double mean = 0;
	for(int i = 0; i<n; i++)
	{
		c.generateGlobule(10000);
		res(i) = c.get_mean_squared_distance();
		mean += res(i);
	}
	mean = mean/n;
	double var = 0;
	for(int i = 0; i<n; i++)
	{
		var += pow(res(i) - mean,2);
	}
	
	var = sqrt(var)/(n-1);

	std::cout << mean<< "  "<< var <<  std::endl;
}

void print_chain_to_file(Chain& c)
{
	std::ofstream file;
	file.open(INITIAL_STATE, std::fstream::out | std::fstream::trunc);
	if(file.is_open())
	{
		file << c << std::endl;
	}
	file.close();
}

