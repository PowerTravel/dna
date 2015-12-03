#ifndef SIMULATION_HPP
#define SIMULATION_HPP


#ifndef PARAM_TYPES
#define PARAM_TYPES

#define UNSIGNED_INTEGER_TYPE 1
#define DOUBLE_TYPE 2
#define STRING_TYPE 3
#define BOOL_TYPE 4
#define MAP_TYPE 5

#endif // PARAM_TYPES

#ifndef VAL_BITS
#define VAL_BITS

// RUN
#define VAL_BIT_VERIFY 1
#define VAL_BIT_VISUALIZE 2
#define VAL_BIT_DISTANCE 4
// TYPE
#define VAL_BIT_PHANTOM 8
#define VAL_BIT_SAW 16
#define VAL_BIT_FG 32

#endif // VAL_BITS

#include <iostream>
#include <map>
#include <string>
#include <fstream>
#include <random>

#include "SAWChain.hpp"
#include "PhantomChain.hpp"
#include "FractalGlobule.hpp"

// A basic simulation class containing general simulation parameters
// Every new simulation will have a new derived simulation class

#ifndef DEFAULT_GENERAL_PARAMETERS
#define DEFAULT_GENERAL_PARAMETERS

#define DEFAULT_TYPE VAL_BIT_FG
#define DEFAULT_OUTFILE "../data/default_data.dna"
#define DEFAULT_VERBOSE true
#define DEFAULT_SELFINT true
#define DEFAULT_WEIGHT  true

#endif // DEFAULT_GENERAL_PARAMETERS
class Simulation
{
	public:
		Simulation();
		Simulation(std::map<std::string, std::string> sm);
		virtual ~Simulation();
		
		bool valid();

		friend std::ostream& operator<<(std::ostream& os, const Simulation& b);
		
		virtual void apply() = 0;
		virtual void print(std::ostream& os)= 0;
	protected:
		Chain * _c;

		bool _valid;
		// General Parameters
		int _chain_type;
		int _simulation_type;
		std::string _outfile;
		bool verbose;
		bool _selfint;
		bool _weight;

		bool text_to_bool(std::string l);
		int text_to_int(std::string l);
		double text_to_double(std::string l);

		void print_pre_info();
		void print_post_info();
		
		const static std::map<std::string, int> param_map;
		const static std::map<std::string, int> val_map;
		const static std::map<int, std::string> dictionary;
	private:



		static std::map<std::string , int> create_value_map();
		static std::map<std::string , int> create_parameter_map();
		static std::map<int, std::string> create_simulation_dictionary();

		
		void set_general_parameters(std::map<std::string, std::string> sm);
		void allocateChain();


		bool is_valid(std::map<std::string, std::string> sm);
		bool check_uint_type(std::string val);
		bool check_double_type(std::string val);
		bool check_string_type(std::string val);
		bool check_bool_type(std::string val);
		bool check_map_type(std::string val);
		bool check_general_map_type(std::string val);

};

inline std::ostream& operator << (std::ostream& os, Simulation* b)
{
	b->print(os);
	return os;
};

#endif // SIMULATION_HPP

