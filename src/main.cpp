#include <iostream>
#include <Eigen/Dense>
#include <random>
#include "Chain.hpp"

int main(int argc, char* argv[])
{
	if(argc>1)
	{	
		std::cout << "Your seed: "<< argv[1] << std::endl;
		srand( atoi(argv[1]) );
	}else{
		std::cout << "Random seed: "<< time(NULL) << std::endl;
		srand(time(NULL));
	}
	
	/* Generate a chain or load chain configuration from file */
	
	/* Simulate the chain */
	Chain c = Chain();
	c.generateGlobule(2);

	/* Print data to file */

	std::default_random_engine generator;
	std::uniform_real_distribution<double> distribution(0.0,1.0);
	for(int i = 0; i<1; i++){
	double ax = distribution(generator);
	std::cout << "Random() " << random() << std::endl;
	std::cout << "  Rand() " << rand() << std::endl;
	std::cout << "Engine() "  << ax << std::endl;
	}
	std::cout << "All OK!" << std::endl;
	return 0;
}
