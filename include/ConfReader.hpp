#ifndef CONF_READER_HPP
#define CONF_READER_HPP

#include <string>
#include <map>
#include <vector>

/*
#define PARAM_BIT_RUN 1
#define PARAM_BIT_TYPE 2
#define PARAM_BIT_OUTFILE 4
#define PARAM_BIT_VERBOSE 8
#define PARAM_BIT_NR_STRIDES 16
#define PARAM_BIT_STRIDE_LEN 32
#define PARAM_BIT_GROWTH 64
#define PARAM_BIT_SAMPLES 128
#define PARAM_BIT_SIZE 256
*/
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
// TYPE
#define VAL_BIT_PHANTOM 4
#define VAL_BIT_SAW 8
#define VAL_BIT_FG 16
// VERBOSE
#define VAL_BIT_TRUE 32
#define VAL_BIT_FALSE 64
#endif // VAL_BITS


class ConfReader{

	public:
		
		struct simulation_config
		{
			// Default values
			simulation_config()
			{
				// General with default
				type = VAL_BIT_PHANTOM;
				outfile = "../data/default_data.dna";
				verbose = true;
				
				// Specifig to verify
				run = 0;
				nr_strides = 0;
				stride_len = 0;
				growth = 0;
				samples = 0;

				// Specific to visualize
				size = 0;

				// meta
				initiated = false;
			}

			int run;
			int type;
			std::string outfile;
			bool verbose;
			int nr_strides;
			int stride_len;
			double growth;
			int samples;
			int size;
			bool initiated;
			
			/// NOT IMPLEMENTED
			bool is_complete(); // Checks if all required parameters are filled
			/// NOT IMPLEMENTED
			friend std::ostream& operator<<(std::ostream& os, const simulation_config& cfg);
			
		};

		ConfReader();
		virtual ~ConfReader();


		void read(std::string filePath = "../configs/DEFAULT");
	private:


		const static std::map<std::string, int> param_map;
		const static std::map<std::string, int> val_map;

		std::string remove_leading_whitespace(std::string line);
		std::string isolate_first_word(std::string line);
		bool is_valid(std::string param, std::string val);

		std::vector< simulation_config > _sim_list;
		void push_back_conf(std::string param, std::string val);
		
		
		static std::map<std::string , int> create_value_map();
		static std::map<std::string , int> create_parameter_map();
		bool check_uint_type(std::string val);
		bool check_double_type(std::string val);
		bool check_string_type(std::string val);
		bool check_bool_type(std::string val);
		bool check_map_type(std::string val);
};



#endif // CONF_READER_HPP 
