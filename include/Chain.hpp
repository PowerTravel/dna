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

#include "Sphere.hpp"
#include <memory>
// Base class for chains
class Chain
{
	public:
		struct link{
			int idx;
			Eigen::Array3d p;
			std::shared_ptr<CollisionGeometry> geom;
		};

		Chain();
		virtual ~Chain();

		void allow_selfintersection( bool as = true);
		void use_weights( bool as = true );
	
		virtual void build(int N) = 0;
		Eigen::ArrayXXd as_array();
		Eigen::ArrayXXd as_array(int start, int size);
		Eigen::ArrayXd weights();
		Eigen::Array3d axis_length();
		double Rg(int start, int size);
		double Rg();
		Eigen::Array3d cm(int start, int size);

		link get_link(int i);

		int len();

		bool ok();

		friend std::ostream& operator<<(std::ostream& os, const Chain& c);
	protected:
		bool _selfint;
		bool _use_weights;
		bool _ok;

		Eigen::ArrayXXd _chain;
		Eigen::ArrayXd _w;
		//double _a;

		//PFloat mult_weights();
		Eigen::Array3d int_to_coord(int i);
		static std::default_random_engine _generator;

};

#endif // RCHAIN_HPP
