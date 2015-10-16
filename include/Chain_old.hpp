#ifndef CHAIN_HPP
#define CHAIN_HPP



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

#include "Sphere.hpp"
#include "Spring.hpp"
class Chain
{
	public:
		Chain();
		virtual ~Chain();
		
		void update(double dt = 0.01);

		// Generera kedjan i en spatial haschmap
		void generateGlobule(int N);
		

		double get_mean_squared_distance(int start, int end);
		double get_mean_squared_distance();
		
		friend std::ostream& operator<<(std::ostream& os, const Chain& c);
		int _redo;
	private:


		//Random Walk
		Eigen::Vector3d getNextStep_random();
		Eigen::Vector3d getNextStep_selfAvoiding();


		struct knot{
			int start;
			int len;
		};

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

		bool _verbose;

		static std::default_random_engine _generator;

		int _N;
		int _n;

		std::vector< link > _globule;
		std::vector< knot > _knots;
		int _knots_check;

		int hash_fun(Eigen::Vector3d x);

		Eigen::Vector3d int_to_coord(int i);
		int coord_to_int(Eigen::Vector3d pos);

		Eigen::Vector3d getNextStep_globule(std::map<int,int>& m);
		Eigen::VectorXd get_stepping_PDF(std::map<int,int>& m);
		Eigen::VectorXd PDF_to_CDF(Eigen::VectorXd f);
		Eigen::Vector3d nextStep(Eigen::VectorXd F);
};

#endif // CHAIN_HPP
