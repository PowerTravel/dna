#include "Verify.hpp"

#include <iostream>
Verify::Verify()
{

}

Verify::Verify(std::map<std::string, std::string> sm) : Simulation(sm)
{
//	std::cout << "Type = " << type << " outfile = " << outfile << " verbose = " << verbose << std::endl;

	_valid = false;

	if( sm.find("NR_STRIDES") != sm.end() )
	{
		_nr_strides  = text_to_int(sm["NR_STRIDES"]);
		_valid = true;
	}
	if( sm.find("STRIDE_LEN") != sm.end() )
	{
		_stride_len  = text_to_int(sm["STRIDE_LEN"]);
		_valid = true;
	}
	if( sm.find("GROWTH") != sm.end() )
	{
		_growth  = text_to_double(sm["GROWTH"]);
		_valid = true;
	}
	if( sm.find("SAMPLES") != sm.end() )
	{
		_samples  = text_to_int(sm["SAMPLES"]);
		_valid = true;
	}
}
Verify::~Verify()
{

}

