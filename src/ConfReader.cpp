#include "ConfReader.hpp"
#include <iostream>
#include <fstream>
#include <errno.h>
const std::map<std::string, int> ConfReader::param_map = 
							ConfReader::create_parameter_map();
const std::map<std::string, int> ConfReader::val_map = 
							ConfReader::create_value_map();

ConfReader::ConfReader()
{

}
ConfReader::~ConfReader()
{

}


void ConfReader::read(std::string filePath)
{
	std::ifstream file;
	errno = 0;
	file.open(filePath);

	bool next_simulation = false;
	if(!file.is_open())
	{
		std::cerr << "ERROR: Could not open '" << filePath << "'"<<std::endl << "Reason: " << strerror(errno) <<"."<< std::endl;
		file.close();
		return;
	}
	
	while(!file.eof())
	{
		char buff[256];
		std::string param, val, line;	
		size_t p = 0;
		
		file.getline(buff, 256, '\n');
		line = buff;

		// Remove leading whitespace
		line = remove_leading_whitespace(line);
		if(line.empty()) continue;

		// Removes comments
		if(line[0] == '%') continue;	

		
		// If the line does not contain a ':' we remove it
		if( (p = line.find_first_of(':')) == std::string::npos) continue;

		param = line.substr(0,p);
		val = line.substr(p+1);

		param = isolate_first_word(param);
		val = isolate_first_word(val);

		// Now we have a param cotaining a word and val containing a word

		if(is_valid(param, val))
		{
			// If the first command is "RUN" it signals a new simulation

			push_back_conf(param,val);

			std::cout << "--|" << param << "|-- : --|" << val <<"|--"<< std::endl;
		}

	}

	std::cout << _sim_list.size() << std::endl;	
	file.close();
}

/// NOT IMPLEMENTED
std::ostream& operator<<(std::ostream& os, const ConfReader::simulation_config& cfg)
{
	return os;
}
// NOT IMPLEMENTED
void ConfReader::push_back_conf(std::string param, std::string val)
{
/*
	if( param.compare("RUN") == 0 )
	{
		//_sim_list.push_back(simulation_config());
	}
	
	if(_sim_list.empty()) return;

	std::map<std::string, int> m = _sim_list.back();
	// We only add a param the first time it appears	
	if(m.find(param) == m.end() )
	{
		int type = param_map.at(param);
		if( type == MAP_TYPE)
		{
			m[param] = val_map.at(val);
		}

		_sim_list.pop_back();
		_sim_list.push_back(m);
	}
*/
}

bool ConfReader::is_valid(std::string param, std::string val)
{

	if(param.empty() || val.empty()) return false;
	if(param_map.find(param) == param_map.end()) return false;

	int data_type = param_map.at(param);
	bool return_flag = false;
	switch(data_type)
	{
		case UNSIGNED_INTEGER_TYPE:
			return_flag = check_uint_type(val);
			break;

		case DOUBLE_TYPE:
			return_flag = check_double_type(val);
			break;

		case STRING_TYPE:
			return_flag = check_string_type(val);
			break;

		case BOOL_TYPE:
			return_flag = check_bool_type(val);
			break;

		case MAP_TYPE:
			return_flag = check_map_type(val);
			break;
		
		default:
			break;
	}

	return return_flag;
}

std::string ConfReader::remove_leading_whitespace(std::string line)
{
	size_t p = line.find_first_not_of("\n\t ");
	if(p == std::string::npos)
	{
		return std::string();
	}
	return line.substr(p);
}

std::string ConfReader::isolate_first_word(std::string line)
{
	line = remove_leading_whitespace(line);
	if(!line.empty())
	{
		size_t p = 0;
		p=line.find_first_of("\n\t ");
		return line.substr(0,p);
	}
	return std::string();
}
		
		
std::map<std::string , int> ConfReader::create_parameter_map()
{
	std::map<std::string, int> m;
	// General
	/*
	m["RUN"] = PARAM_BIT_RUN;
	m["TYPE"] = PARAM_BIT_TYPE;
	m["OUTFILE"] = PARAM_BIT_OUTFILE;
	m["VERBOSE"] = PARAM_BIT_VERBOSE;

	// verify
	m["NR_STRIDES"] = PARAM_BIT_NR_STRIDES;
	m["STRIDE_LEN"] = PARAM_BIT_STRIDE_LEN;
	m["GROWTH"] = PARAM_BIT_GROWTH;
	m["SAMPLES"]= PARAM_BIT_SAMPLES;

	// Visualize
	m["SIZE"] = PARAM_BIT_SIZE;
	*/
	
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

std::map<std::string , int> ConfReader::create_value_map()
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

bool ConfReader::check_uint_type(std::string val)
{
	if(val.find_first_not_of("0123456789") == std::string::npos )
	{
		return true;	
	}else{
		return false;
	}
}
bool ConfReader::check_double_type(std::string val)
{
	if(val.find_first_not_of("0123456789+-.") == std::string::npos )
	{
		return true;	
	}else{
		return false;
	}
}
bool ConfReader::check_string_type(std::string val)
{
	return !val.empty();
}
bool ConfReader::check_bool_type(std::string val)
{
	if( (val.compare("true")==0) || (val.compare("false")==0))
	{
		return true;
	}else{
		return false;
	}
}
bool ConfReader::check_map_type(std::string val)
{
	if(val_map.find(val) != val_map.end())
	{
		return true;
	}else{
		return false;
	}
}
