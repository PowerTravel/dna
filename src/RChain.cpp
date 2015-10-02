#include "RChain.hpp"
#include <iostream>
#include <cmath>
#include <iomanip>
#include <ctime>


std::default_random_engine RChain::_generator = std::default_random_engine(time(NULL));

RChain::RChain()
{
}

RChain::~RChain()
{

}


void RChain::update(double dt)
{

}

void RChain::build(int N)
{
	_N = N;

	// Make sure not to reallocate the _chain vector while g_map is active
	_chain = std::vector< link >();
	
	// Start position
	_chain.push_back(link(0, Eigen::Vector3d(0,0,0)));

	for(int i = 1; i<N; i++)
	{
		_n = i;
		Eigen::Vector3d pos =_chain[i-1].pos + getNextStep();
		
		_chain.push_back(link(i, pos));
	}
}

int RChain::foo()
{
	return 1;
}

Eigen::Vector3d RChain::getNextStep(  )
{
	// Choose the next step acording to  F
	Eigen::VectorXd f = Eigen::VectorXd::Ones(6);
	f = f / f.sum();
	Eigen::VectorXd F = PDF_to_CDF(f);
	
	std::uniform_real_distribution<double> distribution(0.0,1.0);


	double rand_nr = distribution(_generator);
	int i = 0;
	while( !( (rand_nr > F(i)) && (rand_nr <F(i+1) ))  ) 
	{
		i++;	
	}

	return int_to_coord(i);
}

void RChain::random_walk(double* ret)
{
	Eigen::VectorXd f = Eigen::VectorXd::Ones(6);
	f = f / f.sum();
	for(int i = 0; i<6; i++)
	{
		ret[i] = f(i);
	}
}

Eigen::VectorXd RChain::PDF_to_CDF(Eigen::VectorXd f)
{
	int len = f.size();
	Eigen::VectorXd F = Eigen::VectorXd::Zero(len+1);

	for(int i=0; i<len; i++)
	{
		F(i+1) = f.segment(0,i+1).sum();
	}
	return F;
}


Eigen::Vector3d RChain::int_to_coord(int i)
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

int RChain::coord_to_int(Eigen::Vector3d pos)
{
	Eigen::Vector3i p = pos.cast<int>();
	int ret = 0;
	if(p(0) == 1){
		return 0;	
	}else if(p(0) == -1){
		return 1;	
	}else if(p(1) == 1){
		return 2;	
	}else if(p(1) == -1){
		return 3;	
	}else if(p(2) == 1){
		return 4;	
	}else if(p(2) == -1){
		return 5;	
	}
	return ret;
}

int RChain::hash_fun(Eigen::Vector3d x)
{
	return x(0) + MAX_GRID_SIZE*x(1) +MAX_GRID_SIZE*MAX_GRID_SIZE*x(2);
}
		
std::ostream& operator<<(std::ostream& os, const RChain& c)
{
	for( auto ptr = c._chain.begin(); ptr!= c._chain.end(); ptr++)
	{
		Eigen::Vector3d v = ptr->pos;
		os <<std::setw(PRINT_SPACING)<< v(0)<< " " <<std::setw(PRINT_SPACING) << v(1) <<" " << std::setw(PRINT_SPACING) << v(2) << " " << std::endl;	
	}
	return os;
}

double RChain::get_mean_squared_distance()
{
	Eigen::Vector3d len = _chain.back().pos - _chain.front().pos;
	return len.norm();
}
