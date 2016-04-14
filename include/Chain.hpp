#ifndef CHAIN_HPP
#define CHAIN_HPP

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

#include "Sphere.hpp"
#include <memory>
// Base class for chains
class Chain
{
	public:
		Chain();
		virtual ~Chain();

		void allow_selfintersection( bool as = true);
		void use_weights( bool as = true );
	
		virtual void build(int N) = 0;
		ArrXXd as_array();
		ArrXXd as_array(int start, int size);
		ArrXd weights();
		Arr3d axis_length();
		ArrXd span();
		double Rg(int start, int size);
		double Rg();
		Arr3d cm(int start, int size);

		void set_radius(double r);
		void set_link_length(double l);
		void center_chain();

		std::vector< cg_ptr > get_collision_vec();
		std::vector< cg_ptr > get_collision_vec(VecXd boundary);
		VecXd get_density_boundary(double density);


		int len();

		bool ok();

		friend std::ostream& operator<<(std::ostream& os, const Chain& c);
	protected:
		bool _selfint;
		bool _use_weights;
		bool _ok;

		Eigen::ArrayXXd _chain;
		Eigen::ArrayXd _w;
		double _rad;
		double _link_len;

		Eigen::Array3d int_to_coord(int i);
		static std::default_random_engine _generator;
		struct side_density{
			int* wMin;
			int* wMax;
			int* hMin;
			int* hMax;
			int* offset;

			int CO;
			int EO;
			int BO;
	
			int Vol;

			double Density;
	
			bool dirty;
		};		
		int get_3d_array_index(int i, int j, int k, int I, int J, int K);
		int GetSideIdx(int side, int offset, int width, int height, int I,int J, int K);
		void CalcSideDensity(int* box, int I, int J, int K, side_density* s, int CalcSide);


};

#endif // CHAIN_HPP
