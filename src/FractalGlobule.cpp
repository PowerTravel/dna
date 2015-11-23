#include "FractalGlobule.hpp"
#include <iostream> 
FractalGlobule::~FractalGlobule()
{

}
FractalGlobule::FractalGlobule()
{

}

void FractalGlobule::build(int N)
{
	this->SAWChain::build(N);
	//_w = Eigen::ArrayXd::Ones(N);
}

/*
void FractalGlobule::build(int N)
{
	max_grid_size = 2*N;
	if( pow(max_grid_size ,3 ) >= std::numeric_limits<INT_TYPE>::max()	)
	{
		std::cerr << "SAW CHAIN too long, max len is " << pow(std::numeric_limits<INT_TYPE>::max() ,1/3.f)/2 << std::endl; 
		return; 
	}

	bool a = false;

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
		_chain.block(0,_n,3,1) = _chain.block(0,_n-1,3,1) + 
									next_step.segment(1,DIM);
		set_grid(_chain.block(0,_n,3,1));
		if( (next_step(0) == 0) && (a == false))
		{	
		//	a=true;
			std::cout <<  _n <<  "  axax" << std::endl;
		}
	}

	_weight = 1.0;
}
*/
Eigen::Array4d FractalGlobule::get_next_step()
{
	double eps = 0.00001;
	double A = 10000;

	Eigen::Array4d ret(0,0,0,0);
	Eigen::ArrayXd f = eps*Eigen::ArrayXd::Ones(2*DIM);
	int occupied = 0;
	for(int i = 0; i<2*DIM; i++ )
	{
		Eigen::Array3d step = int_to_coord(i);
		step = step + _chain.block(0,_n-1,3,1);
		if(!is_occupied(step))
		{
			f(i) =(double) (1+A*get_occupied_neighbours(step));
		}else{
			occupied ++;
		}
	}

	f = f/f.sum();
	if(occupied == 2*DIM)
	{
//		std::cout << f.transpose() << std::endl;
	}
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

// Algorithm described by that paper lizana knows about
int FractalGlobule::get_occupied_neighbours(Eigen::Array3d step)
{
	Eigen::Array3d s = Eigen::Array3d::Zero();
	double occupied_cites = 0;
	for(int i = 0; i<2*DIM; i++)
	{
		s = step + int_to_coord(i);
		if(is_occupied(s))
		{
			occupied_cites++;
		}
	}
	return  occupied_cites;
}
