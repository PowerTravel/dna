#ifndef RCHAIN_HPP
#define RCHAIN_HPP

#ifndef MAX_GRID_SIZE
#define MAX_GRID_SIZE 1000
#endif // MAX_GRID_SIZE

#ifndef NR_OF_DIRECTIONS
#define NR_OF_DIRECTIONS 6
#endif // NR_OF_DIRECTIONS

#ifndef PRINT_SPACING
#define PRINT_SPACING 4
#endif // PRINT_SPACING

#include <Eigen/Dense>
#include <vector>
#include <random>
#include <string>
#include <fstream>
//#include <functional>

// Base class for chains
class Chain
{
	public:
		Chain();
		virtual ~Chain();
		
		virtual void build(int N) = 0;
		Eigen::ArrayXXd as_array();
		Eigen::ArrayXXd as_array(int start, int size);
		Eigen::ArrayXd weights();
		Eigen::Array3d span();
		double Rg(int start, int size);
		double Rg();
		Eigen::Array3d cm(int start, int size);


		int len();

		bool ok();

		friend std::ostream& operator<<(std::ostream& os, const Chain& c);
	protected:
		bool _ok;

		Eigen::ArrayXXd _chain;
		Eigen::ArrayXd _w;
		//double _a;

		//PFloat mult_weights();
		Eigen::Array3d int_to_coord(int i);
		static std::default_random_engine _generator;

};

#endif // RCHAIN_HPP
