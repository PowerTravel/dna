#include "SAWChain.hpp"
#include <iostream>
#include <climits>


SAWChain::SAWChain()
{
}
SAWChain::~SAWChain()
{

}

void SAWChain::build(int N)
{
	max_grid_size = 2*N;
	if( pow(max_grid_size ,3 ) >= std::numeric_limits<INT_TYPE>::max()	)
	{
		std::cerr << "SAW CHAIN too long, max len is " << pow(std::numeric_limits<INT_TYPE>::max() ,1/3.f)/2 << std::endl; 
		return; 
	}

	int tries = 0;
	int tries_limit = TRIES_LIMIT;
	do{
		tries ++;
		_retry = false;
		_n = 0;
		_w = Eigen::ArrayXd::Zero(N);
		_chain = Eigen::ArrayXXd::Zero(3,N);
		_grid = std::map<INT_TYPE, int>();
		
		_chain.block(0,0,3,1) = Eigen::Array3d(0,0,0);
		_w(0) = 1;
		set_grid(Eigen::Array3d(0,0,0));
	
		for(_n = 1; _n<N; _n++)
		{
			Eigen::Array4d next_step = get_next_step();
			_w(_n) = next_step(0);
			_chain.block(0,_n,3,1) = _chain.block(0,_n-1,3,1) + 
										next_step.segment(1,DIM);
			set_grid(_chain.block(0,_n,3,1));

			if(_retry == true)
			{
				break;
			}
		}
/*
		if(_retry == true)
		{
			std::cout << "Retry " << _n << std::endl;	
		}
*/
	}while( (_retry == true ) && (tries < tries_limit) );
/*
	if(tries > 1)
	{
		std::cout << tries<< std::endl;
	}
*/
	if(tries >= tries_limit)
	{
		std::cerr << "Could not make a chain with "<< tries << " tries. Exiting program" << std::endl;
		exit(1);
	}
}

Eigen::Array4d SAWChain::get_next_step()
{
	Eigen::Array4d ret(0,0,0,0);
	Eigen::ArrayXd f = Eigen::ArrayXd::Zero(2*DIM);
	int occupied = 0;
	for(int i = 0; i<2*DIM; i++ )
	{
		Eigen::Array3d step = int_to_coord(i);
		step = step + _chain.block(0,_n-1,3,1);
		if(is_occupied(step))
		{
			occupied++;
		}else{
			f(i) = 1;
		}
	}	
	if( occupied == 2*DIM )
	{
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
	if(ret(0) == 0)
	{
		_retry = true;
	}

	ret.segment(1,DIM) = int_to_coord(i);
	return ret;
}

double SAWChain::set_weight(int l)
{
	// Cesar Beleno, Kaven Yau, biased sampling algorithm
	return double(l)/(double(2*DIM - 1)); // 5 = 2*#dimensions - 1
}

INT_TYPE SAWChain::pos_to_idx(Eigen::Array3d pos)
{
	Eigen::Array3d p = pos+max_grid_size/2.f;
	INT_TYPE x,y,z;
	x = (INT_TYPE) p(0);
	y = (INT_TYPE) max_grid_size*p(1);
	z = (INT_TYPE) max_grid_size*max_grid_size*p(2);
	return  (x+y+z);
}
bool SAWChain::is_occupied(Eigen::Array3d pos)
{
	INT_TYPE idx = pos_to_idx(pos);
	if(_grid.find(idx) != _grid.end())
	{
		return true;
	}
	return  false;
}

void SAWChain::set_grid(Eigen::Array3d pos)
{
	INT_TYPE idx = pos_to_idx(pos);
	_grid[idx] = 1;
}
/*
PFloat SAWChain::mult_weights(Eigen::ArrayXd w)
{
	PFloat w_tmp = w[0];
	for(int i =1; i<_w.size(); i++)
	{
		if(_w[i] == 0)
		{
			return PFloat();
		}

		w_tmp = w_tmp * w[i];

		if(w_tmp == 0 )
		{
			std::cerr << "WEIGHT ROUNDED TO ZERO IN SAWCHAIN.CPP " << std::endl;
		}
	}
	return w_tmp;
}
*/
