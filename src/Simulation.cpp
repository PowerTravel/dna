#include "Simulation.hpp"
#include <iostream>

const std::map<std::string, int> Simulation::param_map = 
							Simulation::create_parameter_map();
const std::map<std::string, int> Simulation::val_map = 
							Simulation::create_value_map();

Simulation::Simulation()
{

}

Simulation::Simulation(std::map<std::string, std::string> sm)
{
	if( is_valid(sm) )
	{
			
	}
}

Simulation::~Simulation()
{

}
		
void Simulation::apply()
{

}


// Check simulation wide parameters
bool Simulation::is_valid(std::map<std::string, std::string> sm)
{
	bool return_flag = true;
	// For every entery in the simulation_map (sm) containing all parameters
	// and corresponding arguments
	for(auto it = sm.begin(); it != sm.end(); it++)
	{
		// Check if each argument exists in the argument dictionary
		if( param_map.find( it->first ) == param_end.end() )
		{
			std::cerr << "ERROR: " << it->first << " is 	
		}

		int data_type =  param_map.at(it->first);
		switch(data_type)
		{
			case UNSIGNED_INTEGER_TYPE:
				return_flag = check_uint_type(it->second);
				break;
	
			case DOUBLE_TYPE:
				return_flag = check_double_type(it->second);
				break;
	
			case STRING_TYPE:
				return_flag = check_string_type(it->second);
				break;
	
			case BOOL_TYPE:
				return_flag = check_bool_type(it->second);
				break;
	
			case MAP_TYPE:
				return_flag = check_map_type(it->second);
				break;
			
			default:
				break;
		}
	}

	return return_flag;
	//for(auto it2 = sm.begin(); it2!= sm.end(); it2++ )
	//{	
	//	std::cout << it2->first << " : " << it2->second << std::endl;
	//}
}

std::map<std::string , int> Simulation::create_parameter_map()
{
	std::map<std::string, int> m;
	
	m["RUN"] = MAP_TYPE;
	m["TYPE"] = MAP_TYPE;
	m["OUTFILE"] = STRING_TYPE;
	m["VERBOSE"] = BOOL_TYPE;

	// verify
	m["NR_STRIDES"] =UNSIGNED_INTEGER_TYPE;
	m["STRIDE_LEN"] =UNSIGNED_INTEGER_TYPE;
	m["GROWTH"] = DOUBLE_TYPE;
	m["SAMPLES"]= UNSIGNED_INTEGER_TYPE;

	// Visualize
	m["SIZE"] = UNSIGNED_INTEGER_TYPE;

	return m;
}

std::map<std::string , int> Simulation::create_value_map()
{
	std::map<std::string, int> m;
	// RUN
	m["verify"] = VAL_BIT_VERIFY;
	m["visualize"] = VAL_BIT_VISUALIZE;
	//TYPE
	m["phantom"] = VAL_BIT_PHANTOM;
	m["saw"] = VAL_BIT_SAW;
	m["fg"] = VAL_BIT_FG;

	return m;
}

bool Simulation::check_uint_type(std::string val)
{
	if(val.find_first_not_of("0123456789") == std::string::npos )
	{
		return true;	
	}else{
		return false;
	}
}
bool Simulation::check_double_type(std::string val)
{
	if(val.find_first_not_of("0123456789+-.") == std::string::npos )
	{
		return true;	
	}else{
		return false;
	}
}
bool Simulation::check_string_type(std::string val)
{
	return !val.empty();
}
bool Simulation::check_bool_type(std::string val)
{
	if( (val.compare("true")==0) || (val.compare("false")==0))
	{
		return true;
	}else{
		return false;
	}
}
bool Simulation::check_map_type(std::string val)
{
	if(val_map.find(val) != val_map.end())
	{
		return true;
	}else{
		return false;
	}
}
