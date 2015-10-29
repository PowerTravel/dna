#ifndef SAW_CHAIN_HPP
#define SAW_CHAIN_HPP

#include <map>
#include "Chain.hpp"


class SAWChain : public Chain
{
	public:
		SAWChain();
		virtual ~SAWChain();

		virtual void build(int N);
	private:
		Eigen::ArrayXd _w;
		std::map<int,bool> _grid_x;
		std::map<int,bool> _grid_y;
		std::map<int,bool> _grid_z;

		bool rebuild;

		bool is_occupied(Eigen::Array3d pos);
		void set_grid(Eigen::Array3d pos);
		double set_weight(int l);


		Eigen::Array4d get_next_step();
		Eigen::ArrayXd get_PDF();
};

#endif
