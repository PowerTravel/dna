#ifndef SAW_CHAIN_HPP
#define SAW_CHAIN_HPP

#ifndef INT_TYPE
#define INT_TYPE unsigned long long
#endif

#ifndef DIM
#define DIM 3
#endif // DIM

#ifndef TRIES_LIMIT
#define TRIES_LIMIT 1000
#endif

#include <map>
#include "Chain.hpp"

class SAWChain : public Chain
{
	public:
		SAWChain();
		virtual ~SAWChain();

		virtual void build(int N);

	protected:

		bool _retry;
		int max_grid_size;
		int _n;
		std::map<INT_TYPE, int> _grid;

		bool is_occupied(Eigen::Array3d pos);
		void set_grid(Eigen::Array3d pos);
		double set_weight(int l);
	
		INT_TYPE pos_to_idx(Eigen::Array3d pos);
		
		virtual Eigen::Array4d get_next_step();
		Eigen::ArrayXd get_PDF();
//		PFloat mult_weights(Eigen::ArrayXd w);
};

#endif
