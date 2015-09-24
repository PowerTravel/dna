#include "Chain.hpp"
#include <iostream>
#include<cmath>
std::default_random_engine Chain::_generator = std::default_random_engine(time(NULL));

std::map< int, std::string > Chain::_int_dir = Chain::Dir::int_to_dir_map();
std::map< std::string, int > Chain::_dir_int = Chain::Dir::dir_to_int_map();

Chain::Chain()
{
	
}
Chain::Chain(int seed)
{
	_generator = std::default_random_engine(seed);
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
	std::cout << "Generating Globule with " << N << " links " << std::flush;
	_globule = std::map<int, int>();
	
	// Start position
	Eigen::Vector3d pos(0,0,0);
	_globule[hash_fun(pos)] = 6; // Nr 6 is the unique identifier for start position
	int dots = 0;
	for(int i = 1; i<N; i++)
	{
		dots = print_dots(i,N,dots, 5);
		pos = getNewPos(pos);
		_globule[hash_fun(pos)] = coord_to_int(pos);
	}
	std::cout << std::endl;
	//std::cout << pos.transpose() << std::endl;
	std::cout << "RN = " << sqrt(pos.norm()) << "  N = "  << N << " N^(1/3) = " <<pow(N,1.0/3.0) << std::endl;
}

int Chain::print_dots(double i, double N, double printed_dots, double tot_dots)
{
	if(i == floor( (printed_dots+1) * N / (tot_dots+1) ))
	{
		std::cout << "."<<std::flush;
		return printed_dots+1;
	}
	return printed_dots;
}

Eigen::Vector3d Chain::getNewPos(Eigen::Vector3d pos)
{
	Eigen::VectorXd f = get_stepping_PDF(pos);

	Eigen::VectorXd F = PDF_to_CDF(f);

	std::uniform_real_distribution<double> distribution(0.0,1.0);
	double rand_nr = distribution(_generator);
	int i = 0;

	// What happens if rand_nr is exactly 0?
	while( true  )
	{
		if(i >= F.size() )
		{
			std::cout <<  "F.Size:  " << F.size() << std::endl;
			std::cout <<  "F:       " << F.transpose() << std::endl;
			std::cout <<  "Rand Nr: " << rand_nr << std::endl;
		}
		if((rand_nr > F(i)) && ( rand_nr <= F(i+1) ))
		{
			break;
		}
		i++;	
	}
	return pos+int_to_coord(i);
}

Eigen::VectorXd Chain::get_stepping_PDF(Eigen::Vector3d pos)
{
	double A = 10000;
	double epsilon = 0.0001;
	// Porbability Distribution of sites
 	Eigen::VectorXd f(6);
	f << 0,0,0,0,0,0;
	
	// For the six different directions (up down all around)
	int occupied = 0;
	for(int j = 0; j<6; j++)
	{
		// A site next to pos
		Eigen::Vector3d site = pos + int_to_coord(j);

		// If the new site is unoccupied we see how many neigburs it has
		if(_globule.find( hash_fun(site) ) == _globule.end())
		{
			int neighbours = 0;
			for(int k =0; k<6; k++)
			{
				// if the neighbour is occupied
				if( _globule.find( hash_fun(site + int_to_coord(k)) ) != _globule.end() )
				{
					neighbours++;
				}
			}

			f(j) = 1+A*neighbours;
		}else{
			f(j) = epsilon;
			occupied ++;
		}
	}
	// If we are in a place where all neighbouring places are occupied
	// we have a knot
	if(occupied == 6)
	{
		_knots ++;
		//std::cout << "KNOT " << _knots<<  std::endl;
	}
	return f/f.sum();
}

Eigen::VectorXd Chain::PDF_to_CDF(Eigen::VectorXd f)
{
	int len = f.size();
	Eigen::VectorXd F = Eigen::VectorXd::Zero(len+1);
	///F << 0,0,0,0,0,0,0;

	for(int i=0; i<len; i++)
	{
		F(i+1) = f.segment(0,i+1).sum();
	}
	return F;
}











Eigen::Vector3d Chain::int_to_coord(int i)
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

int Chain::coord_to_int(Eigen::Vector3d pos)
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

int Chain::hash_fun(Eigen::Vector3d x)
{
	return x(0) + MAX_GRID_SIZE*x(1) +MAX_GRID_SIZE*MAX_GRID_SIZE*x(2);
}
