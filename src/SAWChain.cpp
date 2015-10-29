#include "SAWChain.hpp"

#define DIM 3
#include <iostream>

SAWChain::SAWChain()
{
	_weight = 0;
}
SAWChain::~SAWChain()
{

}


void SAWChain::build(int N)
{
	rebuild = false;

	do{
		rebuild = false;
		_w = Eigen::ArrayXd::Zero(N);
		_chain = Eigen::ArrayXXd::Zero(3,N);
		_grid_x = std::map<int,bool>();
		_grid_y = std::map<int,bool>();
		_grid_z = std::map<int,bool>();

		
		std::uniform_real_distribution<double> distribution(0,6);
		
		_chain.block(0,0,3,1) = Eigen::Array3d(0,0,0);
		_w(0) = 1;
		_grid_x[0] = true;
		_grid_y[0] = true;
		_grid_z[0] = true;
		for(int i = 1; i<N; i++)
		{
			Eigen::Array4d next_step = get_next_step();
	
			_w(i) = next_step(0);
			_chain.block(0,i,3,1) = _chain.block(0,i-1,3,1) + 
										next_step.segment(1,DIM);
			set_grid(next_step.segment(1,DIM));
		}
	}while(rebuild == true);

	_weight = _w.prod();
	
}

Eigen::Array4d SAWChain::get_next_step()
{
	Eigen::Array4d ret(0,0,0,0);
	Eigen::ArrayXd f = Eigen::ArrayXd::Zero(2*DIM);
	int occupied = 0;
	for(int i = 0; i<2*DIM; i++ )
	{
		Eigen::Array3d step = int_to_coord(i);
		
		if(is_occupied(step))
		{
			std::cout << step.transpose() << std::endl;
			occupied++;
		}else{
			f(i) = 1;
		}
	}	

	if( occupied == 2*DIM )
	{
		rebuild = true;
		return Eigen::Array4d::Zero();
	}

	f = f/f.sum();
	Eigen::ArrayXd F = Eigen::ArrayXd::Zero(2*DIM+1);

	for(int i=0; i<2*DIM; i++)
	{
		F(i+1) = f.segment(0,i+1).sum();
	}

	std::uniform_real_distribution<double> distribution(0.0,1.0);
	double rand_nr = distribution(_generator);

	int i = 0;
	while( !( (rand_nr > F(i)) && (rand_nr <F(i+1) ))  ) 
	{
		i++;	
	}
	ret(0) = set_weight(2*DIM - occupied);
	ret.segment(1,DIM) = int_to_coord(i);
	
	return ret;
}

double SAWChain::set_weight(int l)
{
	// Cesar Beleno, Kaven Yau, biased sampling algorithm
	return double(l)/(double(2*DIM - 1)); // 5 = 2*#dimensions - 1
}

bool SAWChain::is_occupied(Eigen::Array3d pos)
{
	if( ( _grid_x.find(pos(0)) != _grid_x.end() ) &&
		( _grid_y.find(pos(1)) != _grid_y.end() ) &&
		( _grid_z.find(pos(2)) != _grid_z.end() )  )
	{
		return true;
	}
	return  false;
}

void SAWChain::set_grid(Eigen::Array3d pos)
{
	_grid_x[pos(0)] = true;
	_grid_y[pos(1)] = true;
	_grid_z[pos(2)] = true;
}
