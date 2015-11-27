#ifndef VERIFY_HPP
#define VERIFY_HPP

#ifndef THEORETICAL_SLOPES
#define PHANTOM_SLOPE 1/2.f // Polymer Textbook p 18
#define SAW_SLOPE 0.588		// Lizana
#define FG_SLOPE 1/3.f		// Lizana
#endif // THEORETICAL_SLOPES

#ifndef THEORETICAL_FOREFACTORS
#define PHANTOM_R_FF 1			// Polymer Textbook p 18
#define PHANTOM_RG_FF 1/sqrt(6) // Polymer Textbook p 18
#define SAW_R_FF 1				// Need to find out
#define SAW_RG_FF 0.4205		// Polymer textbook p 40, numerical
#define FG_R_FF 1 				// Unknown
#define FG_RG_FF 1				// Unknown
#endif // THEORETICAL_FOREFACTORS

#ifndef START_POINT
#define START_POINT 10
#endif


#include "Simulation.hpp" 
#include "PhantomChain.hpp"
#include "PFloat.hpp"
#include <Eigen/Dense>
/*
	Valid parameters
	"SIZE" = UNSIGNED_INTEGER_TYPE
	"STRIDES" = UNSIGNED_INTEGER_TYPE;
	"EXP" = BOOL_TYPE;
	"SAMPLES" = UNSIGNED_INTEGER_TYPE;

	Output:
	A file with columns:
	Nr links,  
	R, Variance, Theoretical
	Rg, Variance, Theoretical
*/

class Verify : public  Simulation{
	
	public:
		Verify();
		Verify(std::map<std::string, std::string> sm);
		virtual ~Verify();

		virtual void print(std::ostream& os);
		void apply();
		
	private:
		int _size;
		int _strides;
		bool _exp;
		int _samples;
	
		double _step_size;


		//Chain::ChainType _t;
		double _theoretical_slope;
		double _theoretical_Rg_slope;
	
		void set_plot_points();
		void set_theoretical_values();
		
		// Global
		Eigen::ArrayXd nr_links;    // Number of links in each chain
		Eigen::ArrayXd link_mean;

		// Result arrays for mean distance
		Eigen::ArrayXd R;  			// End to end distance of chains
		Eigen::ArrayXd R_var;		// Variance
		Eigen::ArrayXd R_theo;   	// Theoretical value

		// Result arrays for mean Radious of gyration
		Eigen::ArrayXd Rg;   		// Radius of gyration
		Eigen::ArrayXd Rg_var; 	// Variance
		Eigen::ArrayXd Rg_theo; 	// Theoretical value

		// write progress to terminal
		bool _verbose;					// Plot to terminal?
		double _increment;				// The update ammount
		double _plot_interval; 		    // update rate = interval * increment;
		double _percent;				// Counter


		void init_simulaition_parameters(std::map<std::string, std::string> sm);
		
		void init_plotting_parameters();
		void write_to_terminal(int N, int i, int j);
		void write_to_file();
		void set_chain_type();


		Eigen::Vector2d get_mean_and_variance(Eigen::ArrayXd in_data, std::vector<PFloat >& weight );

		PFloat mult_weights(Eigen::ArrayXd w);
};

#endif //  VERIFY_HPP
