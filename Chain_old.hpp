
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
#include <map>
#include <string>
#include <fstream>


#include <functional>


#include "Sphere.hpp"
#include "Spring.hpp"
class Chain
{
	public:
		enum ChainType{
			PHANTOM,
			SAW,
			FG
		};

		Chain();
		virtual ~Chain();
		
		void update(double dt = 0.01);

		void build(int N, ChainType c = FG);
		
		double get_mean_squared_distance();
		double get_mean_squared_distance(int start, int end);
		Eigen::Vector2d get_binned_mean_square_distance(int start, int end);
		double Rg();
		double Rg(int start, int end);
		Eigen::Vector3d get_CM();
		Eigen::Vector3d get_CM(int start, int end);

		Eigen::ArrayXXd as_array(int start, int end);
		Eigen::ArrayXXd as_array();
		
		friend std::ostream& operator<<(std::ostream& os, const Chain& c);
	private:


		struct link{
			link(){
				w = 1;
				pos = Eigen::Vector3d(0,0,0);
			};
			link(Eigen::Vector3d p, int l = 5)
			{
				//if(_ct == ChainType::SAW){
					w = set_weight(l);
				//}else{
				//	w = 1;
				//}
				pos = p;
			};

			double w; // Weight
			Eigen::Vector3d pos;
			double set_weight(int l)
			{
				// Cesar Beleno, Kaven Yau, biased sampling algorithm
				return double(l)/(5.f); // 5 = 2*#dimensions - 1
			}
		};

		static std::default_random_engine _generator;

		ChainType _ct;
		int _N;
		int _n;

		Eigen::ArrayXd path_chosen;

		std::vector< link > _chain;
		std::map<long long,bool> _grid;

		long long hash_fun(Eigen::Vector3d x);
		Eigen::Vector3d int_to_coord(int i);


		Eigen::Vector3d getNextStep();

		Eigen::VectorXd PDF_to_CDF(Eigen::VectorXd f);
	

		Eigen::VectorXd get_pdf();
		Eigen::VectorXd fractal_globule();
		Eigen::VectorXd self_avoiding();
};

#endif // RCHAIN_HPP
