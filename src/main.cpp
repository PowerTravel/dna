#include <iostream>
#include <vector>
#include <memory>
#include "ConfReader.hpp"
#include "Simulation.hpp"

/*
 * 	Program dna
 *	Generates a fractal globule and prints data to a file
 */


int main(int argc, char* argv[])
{
	std::string config;
	if(argc > 1)
	{
		config = argv[1];
	}else{
		config = "../configs/DEFAULT";
	}

	std::vector< std::shared_ptr<Simulation> > sim_list =  ConfReader::read(config);

	for(auto it = sim_list.begin(); it != sim_list.end(); it++)
	{
		std::shared_ptr<Simulation> p = NULL;	
		p = *it;
		p->apply();
	}
	return 0;
}
