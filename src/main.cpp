#include <iostream>
//#include <Eigen/Dense>
#include <random>
#include "Chain.hpp"

#ifndef OUT_FILE
#define OUT_FILE "../matlab/globule_data.m"
#endif // OUT_FILE


void print_chain_to_file(Chain& c);

int main(int argc, char* argv[])
{
	int N = 10;
	if(argc >= 2)
	{
		N = atoi(argv[1]);
	}
	// Generate a chain or load chain configuration from file 
	
	//Simulate the chain 
	Chain c = Chain();
	c.generateGlobule(N);

	print_chain_to_file(c);
	//c.print_knots();
	 //Print data to file 
	
	return 0;
}
	
void print_chain_to_file(Chain& c)
{
	std::ofstream file;
	file.open(OUT_FILE, std::fstream::out | std::fstream::trunc);
	if(file.is_open())
	{
		file << c << std::endl;
	}
	file.close();
}

