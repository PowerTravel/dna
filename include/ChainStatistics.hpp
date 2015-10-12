#ifndef CHAIN_STATISTICS_HPP
#define CHAIN_STATISTICS_HPP

#ifndef THEORETICAL_SLOPES
#define PHANTOM_SLOPE 1/2.f
//#define SAW_SLOPE 3/(2.f+3.f)
#define SAW_SLOPE 0.588
#define GLOBULE_SLOPE 1/3.f
#endif // THEORETICAL_SLOPES

#ifndef OUT_FILES
#define INITIAL_STATE "../data/initial_state.m"
#define ANIMATION = "../data/animation.m"
#define MEAN_SQUARE_DISTANCE "../data/meanSquareDistance.m"
#endif // OUT_FILES

#include "Eigen/Dense"
#include <string>
class ChainStatistics{
	
	public:
		ChainStatistics();	
		ChainStatistics(std::string confFile);
		virtual ~ChainStatistics();	

		void generate();

	private:

		int _type; // 1 = Phantom, 2 = SAW, 3 = Globule

		// Simulation Parameters
		int _nr_strides;			// Number of data points
		int _stride_len;			// the distance in links between points
		int _samples_per_stride;	// the number of samples for each data point
		double _growth_rate;		// The growth betwen data points

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
		
		void init_plotting_parameters();
		void write_to_terminal(int N, int i, int j);
		void write_to_file();
};

#endif //CHAIN_STATISTICS_HPP 
