#ifndef VERIFY_HPP
#define VERIFY_HPP

#ifndef THEORETICAL_SLOPES
#define PHANTOM_SLOPE 1/2.f
//#define SAW_SLOPE 3/(2.f+3.f)
#define SAW_SLOPE 0.588
#define GLOBULE_SLOPE 1/3.f
#endif // THEORETICAL_SLOPES

#include "Simulation.hpp" 
#include "RChain.hpp"
#include <Eigen/Dense>
/*
	Valid parameters
	"NR_STRIDES" = UNSIGNED_INTEGER_TYPE;
	"STRIDE_LEN" = UNSIGNED_INTEGER_TYPE;
	"GROWTH" = DOUBLE_TYPE;
	"SAMPLES" = UNSIGNED_INTEGER_TYPE;
*/

class Verify : public  Simulation{
	
	public:
		Verify();
		Verify(std::map<std::string, std::string> sm);
		virtual ~Verify();

		virtual void print(std::ostream& os);
		void apply();
		
	private:
		int _nr_strides;
		int _stride_len;
		double _growth;
		int _samples;

		RChain::ChainType _t;
		double _theoretical_slope;
		
		
		// Global
		Eigen::ArrayXd nr_links;    // Number of links in each chain
		
		// Result arrays for mean distance
		Eigen::ArrayXd mDist;  			// End to end distance of chains
		Eigen::ArrayXd mDist_var;		// Variance
		Eigen::ArrayXd mDist_theo;   	// Theoretical value
		Eigen::ArrayXd mDist_err;		// Log of Error (for measuring slope)

		// Result arrays for mean Radious of gyration
		Eigen::ArrayXd mRadGyr;   		// Radius of gyration
		Eigen::ArrayXd mRadGyr_var; 	// Variance
		Eigen::ArrayXd mRadGyr_teo; 	// Theoretical value
		Eigen::ArrayXd mRadGyr_err; 	// Log of deviation from theoretical

		// Result arrays for center of mass
		Eigen::ArrayXd CM;				// Center of mass
		Eigen::ArrayXd CM_var;			// Variance
		Eigen::ArrayXd CM_teo; 			// Theoretical
		Eigen::ArrayXd CM_err;  		// Error

		// write progress to terminal
		bool _verbose;					// Plot to terminal?
		double _increment;				// The update ammount
		double _plot_interval; 		    // update rate = interval * increment;
		double _percent;				// Counter


		void init_arrays();
		void init_simulaition_parameters(std::map<std::string, std::string> sm);
		
		void init_plotting_parameters();
		void write_to_terminal(int N, int i, int j);
		void write_to_file();
		void set_chain_type();
};

#endif //  VERIFY_HPP
