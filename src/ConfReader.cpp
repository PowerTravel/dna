#include "ConfReader.hpp"
#include <iostream>
#include <fstream>
#include <errno.h>
ConfReader::ConfReader()
{

}
ConfReader::~ConfReader()
{

}


std::vector< std::shared_ptr<Simulation> > ConfReader::read(std::string filePath)
{
	std::ifstream file;
	errno = 0;
	file.open(filePath);

	bool next_simulation = false;
	if(!file.is_open())
	{
		std::cerr << "ERROR: Could not open '" << filePath << "'"<<std::endl << "Reason: " << strerror(errno) <<"."<< std::endl;
		file.close();
		return _sim_list;
	}
	
	std::vector< std::map< std::string, std::string > > parsed_config_vec;
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

		// if(is_valid(param, val)) // If valid should be handled by individual simulation classes
		// If we have something in param and val we proceed in creating a list
		if( !param.empty() && !val.empty() )
		{
			// If the first command is "RUN" it signals a new simulation

			if(param.compare("RUN") == 0)
			{
				parsed_config_vec.push_back(std::map< std::string, std::string >() );
			}
	
			if( !parsed_config_vec.empty() )
			{
				std::map< std::string, std::string > m = parsed_config_vec.back();
				m[param] = val;
				parsed_config_vec.pop_back();
				parsed_config_vec.push_back(m);
			}
		}
	}

	for(auto it = parsed_config_vec.begin(); it!= parsed_config_vec.end(); it++ )
	{
		std::map<std::string, std::string> sm = *it;
		if(sm["RUN"].compare("verify")==0 )
		{
			_sim_list.push_back( std::shared_ptr<Simulation>(new Verify(sm)) );
		}else if(sm["RUN"].compare("visualize") == 0)
		{
			_sim_list.push_back(std::shared_ptr<Simulation>(new  Visualize(sm) ) );
		}
		//std::cout << _sim_list.back() << std::endl;
	}

	file.close();

	return _sim_list;
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

/*
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
*/

