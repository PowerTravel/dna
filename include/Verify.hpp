#ifndef VERIFY_HPP
#define VERIFY_HPP

#ifndef THEORETICAL_SLOPES
#define PHANTOM_SLOPE 1/2.f
#define SAW_SLOPE 0.588
#define GLOBULE_SLOPE 1/3.f
#endif // THEORETICAL_SLOPES

#include "Simulation.hpp" 
#include "Chain.hpp"
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
	End distance, Variance, Theoretical, Relative error
	Radius of gyration, Variance, Theoretical, Relative error
	Center of mass, Variance, Theoretical, Relative error
	

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
		double _growth_factor;
		int _start_link;


		Chain::ChainType _t;
		double _theoretical_slope;
		double _theoretical_Rg_slope;
		
		
		// Global
		Eigen::ArrayXd nr_links;    // Number of links in each chain
		
		// Result arrays for mean distance
		Eigen::ArrayXd mDist;  			// End to end distance of chains
		Eigen::ArrayXd mDist_var;		// Variance
		Eigen::ArrayXd mDist_theo;   	// Theoretical value
		Eigen::ArrayXd mDist_err;		// Relative Error

		// Result arrays for mean Radious of gyration
		Eigen::ArrayXd mRadGyr;   		// Radius of gyration
		Eigen::ArrayXd mRadGyr_var; 	// Variance
		Eigen::ArrayXd mRadGyr_theo; 	// Theoretical value
		Eigen::ArrayXd mRadGyr_err; 	// Relative Error

		// Result arrays for center of mass
		Eigen::ArrayXXd CM;				// Center of mass
		Eigen::ArrayXXd CM_var;			// Variance
		Eigen::ArrayXXd CM_theo; 		// Theoretical
		Eigen::ArrayXXd CM_err;  		// Relative Error

		// write progress to terminal
		bool _verbose;					// Plot to terminal?
		double _increment;				// The update ammount
		double _plot_interval; 		    // update rate = interval * increment;
		double _percent;				// Counter


		void init_arrays();
		void set_linear_links();
		void set_exponential_links();
		void init_simulaition_parameters(std::map<std::string, std::string> sm);
		
		void init_plotting_parameters();
		void write_to_terminal(int N, int i, int j);
		void write_to_file();
		void set_chain_type();


		Eigen::Vector2d get_mean_and_variance(Eigen::ArrayXd in_data );
};

#endif //  VERIFY_HPP
