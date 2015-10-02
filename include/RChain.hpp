
#ifndef RCHAIN_HPP
#define RCHAIN_HPP



#ifndef MAX_GRID_SIZE
#define MAX_GRID_SIZE 10000
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
#include <map>
#include <string>
#include <fstream>


#include <functional>


#include "Sphere.hpp"
#include "Spring.hpp"
class RChain
{
	public:
		RChain();
		virtual ~RChain();
		
		void update(double dt = 0.01);

		void build(int N);
		

		double get_mean_squared_distance(int start, int end);
		double get_mean_squared_distance();
		
		friend std::ostream& operator<<(std::ostream& os, const RChain& c);
	private:

		struct link{
			link(){
				nr=0;
				pos = Eigen::Vector3d(0,0,0);
			};
			link(int n, Eigen::Vector3d p)
			{
				nr = n;
				pos = p;
			};
		
			int nr;
			Eigen::Vector3d pos;
		};

		static std::default_random_engine _generator;

		int _N;
		int _n;

		std::vector< link > _chain;

		int hash_fun(Eigen::Vector3d x);

		Eigen::Vector3d int_to_coord(int i);
		int coord_to_int(Eigen::Vector3d pos);

		//Eigen::Vector3d getNextStep( std::function<Eigen::VectorXd()> CDF);
	//	Eigen::Vector3d getNextStep( std::function<void (double*)> CDF);
		Eigen::Vector3d getNextStep( );
		Eigen::VectorXd PDF_to_CDF(Eigen::VectorXd f);
	
		int foo();


		void random_walk(double*);
};

#endif // RCHAIN_HPP
