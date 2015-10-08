#ifndef CONF_READER_HPP
#define CONF_READER_HPP

#include <string>
#include <map>
#include <vector>
#include "Simulation.hpp"

class ConfReader{

	public:
		ConfReader();
		virtual ~ConfReader();


		void read(std::string filePath = "../configs/DEFAULT");
	private:

		std::vector< Simulation > _sim_list;


		std::string remove_leading_whitespace(std::string line);
		std::string isolate_first_word(std::string line);

		// The individual simulations should determine the validity of the input
		// Handled in derived Simulation classes
//		bool is_valid(std::string param, std::string val);
		
		
};



#endif // CONF_READER_HPP 
