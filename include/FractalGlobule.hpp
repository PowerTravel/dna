#ifndef FRACTAL_GLOBULE_HPP
#define FRACTAL_GLOBULE_HPP

#include "SAWChain.hpp"

class FractalGlobule : public SAWChain{

	public:
		FractalGlobule();
		virtual ~FractalGlobule();
		
		virtual void build(int N);

	private:
		int get_occupied_neighbours(Eigen::Array3d step);
		virtual Eigen::Array4d get_next_step();
		
};

#endif // FRACTAL_GLOBULE_HPP
