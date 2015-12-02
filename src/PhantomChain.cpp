#include "PhantomChain.hpp"
#include <iostream>

PhantomChain::PhantomChain()
{
}

PhantomChain::~PhantomChain()
{
}

void PhantomChain::build(int N)
{
	_w = Eigen::ArrayXd::Ones(N);
	_chain = Eigen::ArrayXXd::Zero(3, N);
	
	// Start position
	_chain.block(0,0,3,1) = Eigen::Array3d(0,0,0);
	
	std::uniform_real_distribution<double> distribution(0,6);
	
	for(int i = 1; i<N; i++)
	{
		int step = int(distribution(_generator));
		Eigen::Array3d next_step = int_to_coord(step);
		_chain.block(0,i,3,1) = _chain.block(0,i-1,3,1) + next_step;
	}

	_ok = true;
}
