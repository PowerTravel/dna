#include "Chain.hpp"
#include <iostream>

Chain::Chain()
{

}

Chain::Chain(std::map<int, int> m)
{
	/* Variables */
}

Chain::~Chain()
{

}

void Chain::generateGlobule(int N)
{
	
}

int Chain::spatial_hash_key_fun(int x, int y, int z, int max)
{
	return x + max*y + max*max*z;
}

void Chain::update(double dt)
{
}

Eigen::VectorXd Chain::getPos()
{
	Eigen::VectorXd ret = Eigen::VectorXd(3*_N);
	int i = 0;
	for(auto it = _links.begin(); it != _links.end(); it++ )
	{
		ret.segment(3*i,3) = it->getPos();
		i++;
	}

	return ret;
}

Eigen::VectorXd Chain::getVel()
{
	Eigen::VectorXd ret = Eigen::VectorXd(3*_N);
	int i = 0;
	for(auto it = _links.begin(); it != _links.end(); it++ )
	{
		ret.segment(3*i,3) = it->getVel();
		i++;
	}
	return ret;

}

void Chain::printChain()
{
	std::cout << "==Printing Chain==" << std::endl;
	for(int i=0; i<_N; i++)
	{
		std::cout << _links[i].getPos().transpose() << std::endl;
	}
}
