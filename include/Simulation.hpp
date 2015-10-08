#ifndef SIMULATION_HPP
#define SIMULATION_HPP

#ifndef PARAM_TYPES
#define PARAM_TYPES

#define UNSIGNED_INTEGER_TYPE 1
#define DOUBLE_TYPE 2
#define STRING_TYPE 3
#define BOOL_TYPE 4
#define MAP_TYPE 5

#ifndef VAL_BITS
#define VAL_BITS

// RUN
#define VAL_BIT_VERIFY 1
#define VAL_BIT_VISUALIZE 2
// TYPE
#define VAL_BIT_PHANTOM 4
#define VAL_BIT_SAW 8
#define VAL_BIT_FG 16

#endif // VAL_BITS

#include <map>
#include <string>

#endif // PARAM_TYPES
// A basic simulation class containing general simulation parameters
// Every new simulation will have a new derived simulation class

class Simulation
{
	public:
		Simulation();
		Simulation(std::map<std::string, std::string> sm);
		virtual ~Simulation();
		
		virtual void apply();

	protected:


	private:

		const static std::map<std::string, int> param_map;
		const static std::map<std::string, int> val_map;
		const static std::map<std::string, int> general_param_map;

		static std::map<std::string , int> create_value_map();
		static std::map<std::string , int> create_parameter_map();

		bool is_valid(std::map<std::string, std::string> sm);
		bool check_uint_type(std::string val);
		bool check_double_type(std::string val);
		bool check_string_type(std::string val);
		bool check_bool_type(std::string val);
		bool check_map_type(std::string val);
		bool check_general_map_type(std::string val);
};

#endif // SIMULATION_HPP

