#include <iostream>
//#include <Eigen/Dense>
#include <random>
#include "Chain.hpp"

int main(int argc, char* argv[])
{
	int N = 10000;
	if(argc >= 2)
	{
		N = atoi(argv[1]);
	}
	// Generate a chain or load chain configuration from file 
	
	//Simulate the chain 
	Chain c = Chain();
	c.generateGlobule(N);

	 //Print data to file 
	
	return 0;
}
