#include "Simulation.hpp"
#include <iostream>
#include <ctime>

//std::default_random_engine Simulation::_generator = std::default_random_engine(time(NULL));
const std::map<std::string, int> Simulation::param_map = 
							Simulation::create_parameter_map();
const std::map<std::string, int> Simulation::val_map = 
							Simulation::create_value_map();

const std::map<int, std::string> Simulation::dictionary =
							Simulation::create_simulation_dictionary();
Simulation::Simulation()
{
	_valid = false;
	_c = NULL;
}

Simulation::Simulation(std::map<std::string, std::string> sm)
{
	// Checks that all commands are valid
	_c = NULL;
	_valid =  is_valid(sm);
	if(_valid){

		set_general_parameters(sm);
	}
}

Simulation::~Simulation()
{
	if(_c!= NULL)
	{
		delete _c;
		_c = NULL;
	}
}
		
void Simulation::set_general_parameters(std::map<std::string, std::string> sm)
{
	if( sm.find("TYPE") != sm.end())
	{
		_chain_type = val_map.at(sm["TYPE"]);
		allocateChain();
	}else{
		_chain_type = DEFAULT_TYPE;
		allocateChain();
	}

	if( sm.find("OUTFILE") != sm.end())
	{
		_outfile = sm["OUTFILE"];
	}else{
		_outfile = DEFAULT_OUTFILE;
	}

	if( sm.find("VERBOSE") != sm.end())
	{
		verbose = text_to_bool(sm["VERBOSE"]);
	}else{
		verbose = DEFAULT_VERBOSE;
	}
}

void Simulation::allocateChain()
{
		switch(_chain_type)
		{
			case VAL_BIT_PHANTOM: 
				_c = new PhantomChain();
				break;
		
			case VAL_BIT_SAW:
				_c = new SAWChain();
				break;
		
			case VAL_BIT_FG:
				_c = new FractalGlobule();
				//_c = NULL;
				//std::cout << "In Simulation.src:allocateChain - FractalGlobule not implemented"<< std::endl;
				break;
			default:
				std::cout << "In Simulation.src:allocateChain: Weird Chain TYpe, this line should never be written" << std::endl;
				break;
				
		}
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
		if( param_map.find( it->first ) == param_map.end() )
		{
			std::cerr << "ERROR: " << it->first << " is not a valid parameter " << std::endl;
			return false;
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
		if(return_flag == false)
		{
			std::cerr << "ERROR: '" << it->second << "' is not a valid value for parameter '" << it->first <<"'"<< std::endl;
			return false;
		}
	}

	return true;
}

std::map<int, std::string> Simulation::create_simulation_dictionary()
{
	std::map<int, std::string> m;
	m[VAL_BIT_VERIFY] = "verify";
	m[VAL_BIT_VISUALIZE] = "visualize";
	m[VAL_BIT_PHANTOM] = "Phantom Chain";
	m[VAL_BIT_SAW] = "Self Avoiding Walk";
	m[VAL_BIT_FG] = "Fractal Globule";
	return m;
}
std::map<std::string , int> Simulation::create_parameter_map()
{
	std::map<std::string, int> m;
	// General	
	m["RUN"] = MAP_TYPE;
	m["TYPE"] = MAP_TYPE;
	m["OUTFILE"] = STRING_TYPE;
	m["VERBOSE"] = BOOL_TYPE;

	// verify
	m["SIZE"] = UNSIGNED_INTEGER_TYPE;  // Visualize also
	m["STRIDES"] = UNSIGNED_INTEGER_TYPE;
	m["EXP"] = BOOL_TYPE;
	m["SAMPLES"]= UNSIGNED_INTEGER_TYPE;

	// Visualize
//	m["SIZE"] = UNSIGNED_INTEGER_TYPE;

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

bool Simulation::text_to_bool(std::string l)
{
	if( l.compare("true") == 0 )
	{
		return true;
	}else if(l.compare("false") == 0){
		return false;
	}else{
		std::cerr << "ERROR in Simulation::text_to_bool" << std::endl;
		std::cerr << l << "  Must be 'true' or 'false'. Shutting Down." << std::endl;
		exit(1);
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

int Simulation::text_to_int(std::string l)
{
	return std::stoi(l);
}

double Simulation::text_to_double(std::string l)
{
	return std::stod(l);
}
bool Simulation::valid()
{
	return _valid;
}

void Simulation::print_pre_info()
{
	if(verbose){
		std::cout << "Starting Simulation '" << dictionary.at(_simulation_type) << "' on a '" << dictionary.at(_chain_type) <<"'." << std::endl;
	}
}

void Simulation::print_post_info()
{
	if(verbose){
		std::cout << "Data written to '" << _outfile << "'." << std::endl;
	}
}

