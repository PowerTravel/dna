#include "Chain.hpp"
#include <iostream>

Chain::Chain()
{

}


Chain::~Chain()
{

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


void Chain::generateGlobule(int N)
{
	_globule = std::map<int, Trace>();
	
	// Start position
	Eigen::Vector3d pos(0,0,0);
	_globule.insert(std::pair<int, Trace >(hash_fun(pos), Trace::START ));
	for(int i = 1; i<N; i++)
	{
		Eigen::VectorXd F = get_prob_dist(pos);

	}
}

Eigen::VectorXd Chain::get_prob_dist(Eigen::Vector3d pos)
{
	double A = 10000;
	double epsilon = 0.0001;
	// Porbability Distribution of cites
 	Eigen::VectorXd f(6);
	f << 0,0,0,0,0,0;
	
	// For the six different directions (up down all around)
	for(int j = 0; j<6; j++)
	{
		// We check how many neighbours that cite has
		Eigen::Vector3d cite = pos + get_idx(j);
		int neighbours = 0;
		for(int k =0; k<6; k++)
		{
			if( _globule.find( hash_fun(cite + get_idx(k)) ) != _globule.end() )
			{
				neighbours++;
			}
		}

		if( neighbours != 0 )
		{
			f(j) = 1+A*neighbours;
		}else{
			f(j) = epsilon;
		}
	}

	f = f/f.sum();
	Eigen::VectorXd F(6);
	F << 0,0,0,0,0,0;
	for(int i=0; i<6; i++)
	{
		F(i) = f.segment(0,i+1).sum();
	}
	return F;
}


Eigen::Vector3d Chain::get_idx(int i)
{
	Eigen::Vector3d idx(0,0,0);
	switch(i)
	{
		// POSITIVE X (RIGHT)
		case 0:
			idx << 1,0,0;
			break;
		// NEGATIVE X (LEFT)
		case 1:
			idx << -1,0,0;
			break;
		// POSITIVE Y(FORWARD)
		case 2:
			idx << 0,1,0;
			break;
		// NEGATIVE Y (BACKWARD)
		case 3:
			idx << 0,-1,0;
			break;
		// POSITIVE Z (UP)
		case 4: 
			idx << 0,0,1;
			break;
		// NEGATIVE Z (DOWN)
		case 5:
			idx << 0,0,-1;
			break;
		// STAY
		default:
			idx << 0,0,0;
			break;
	}
	return idx;
}


int Chain::hash_fun(int x, int y, int z)
{
	return x + _max*y + _max*_max*z;
}

int Chain::hash_fun(Eigen::Vector3d x)
{
	return x(0) + _max*x(1) + _max*_max*x(2);
}
