#include "Visualize.hpp"
#include "Chain.hpp"
#include <iostream>
Visualize::Visualize()
{

}

Visualize::Visualize(std::map<std::string, std::string> sm) : Simulation(sm)
{
	_simulation_type = val_map.at("visualize");

	if( sm.find("SIZE") != sm.end() )
	{
		_size  = text_to_int(sm["SIZE"]);
	}else{
		_valid = false;
	}
}
Visualize::~Visualize()
{

}

void Visualize::apply()
{
	print_pre_info();
	if(!_valid)
	{
		std::cerr << "Visualize not valid. Exiting" << std::endl;
		return;
	}

	_c->build(_size);
	// Print to file
	
	std::ofstream file;
	file.open(_outfile, std::fstream::out | std::fstream::trunc);
	if(file.is_open())
	{
		file << *_c << std::endl;
	}
	file.close();

	print_post_info();

}

void Visualize::print(std::ostream& os)
{
	os <<"Simulation = Visualize" << std::endl;
	if(_valid)
	{
		os <<"Out File   = " << _outfile << std::endl;
		os <<"Size = " << _size << std::endl;
	}else{
		os << "Simulation failed to load";
	}
}
