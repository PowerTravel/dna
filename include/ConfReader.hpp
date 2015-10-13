#ifndef CONF_READER_HPP
#define CONF_READER_HPP

#include <string>
#include <map>
#include <vector>
#include <memory>

class Simulation;
class ConfReader{

	public:
		ConfReader();
		virtual ~ConfReader();

		static std::vector< std::shared_ptr<Simulation> > read(std::string filePath);
	private:

		static std::string remove_leading_whitespace(std::string line);
		static std::string isolate_first_word(std::string line);
};



#endif // CONF_READER_HPP 
