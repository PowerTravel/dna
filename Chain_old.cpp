#include "Chain.hpp"
#include <iostream>
#include <cmath>
#include <iomanip>
#include <ctime>
#include <limits>

std::default_random_engine Chain::_generator = std::default_random_engine(time(NULL));

Chain::Chain()
{
}

Chain::~Chain()
{

}


void Chain::update(double dt)
{

}

void Chain::build(int N, ChainType c)
{
	path_chosen = Eigen::ArrayXd::Zero(6);

	_N = N;
	_ct = c;

	_chain = std::vector< link >();
	_grid = std::map<long long,bool>();
	
	// Start position
	_chain.push_back(link(Eigen::Vector3d(0,0,0)));
	_grid[hash_fun(_chain.back().pos)] = 1;

	for(int i = 1; i<N; i++)
	{
		_n = i;
		Eigen::Vector3d pos =_chain.back().pos + getNextStep();
//		std::cout << "position " << pos.transpose() << " Chosen" << std::endl;	
		_chain.push_back(link(pos));
		_grid[hash_fun(pos)] = true;
	}
	//std::cout <<_grid.size() <<" - " << _chain.size() << " = " << ((int)_grid.size()) - ((int) _chain.size()) << std::endl;
//	std::cout << (path_chosen / path_chosen.sum()).transpose() << std::endl;
}

Eigen::Vector3d Chain::getNextStep(  )
{
	Eigen::VectorXd f = get_pdf();
	Eigen::VectorXd F = PDF_to_CDF(f);
	
	std::uniform_real_distribution<double> distribution(0.0,1.0);


	double rand_nr = distribution(_generator);
	int i = 0;
	while( !( (rand_nr > F(i)) && (rand_nr <F(i+1) ))  ) 
	{
		i++;	
	}
	path_chosen[i]++;

	return int_to_coord(i);
}

Eigen::VectorXd Chain::get_pdf()
{
	Eigen::VectorXd f;
	//Random Walk
	if(_ct == ChainType::PHANTOM)
	{
		f = Eigen::VectorXd::Ones(NR_OF_DIRECTIONS);
		f = f / f.sum();
	}
	// Self avoiding random walk
	else if(_ct == ChainType::SAW)
	{
		f = self_avoiding();
	}
	// Fractal Globule
	else if(_ct == ChainType::FG){
	//	std::cerr << "FRACTAL_GLOBULE NOT IMPLEMENTED"<< std::endl;	
	//	exit(1);
		f = fractal_globule();
	}else{
		std::cerr <<_ct <<" is an invalid chainType. Creating Phantom"<< std::endl;	
		exit(1);
	}

	return f;
}

Eigen::VectorXd Chain::self_avoiding()
{
	double A = 100000;
	double eps = 0.0001;
	// Porbability Distribution of sites
 	Eigen::VectorXd f = Eigen::VectorXd::Zero(NR_OF_DIRECTIONS);

	Eigen::VectorXd pos = _chain.back().pos;

	// For the six different directions (up down all around)
	for(int j = 0; j<NR_OF_DIRECTIONS; j++)
	{
		// A site next to pos
		Eigen::Vector3d site = pos + int_to_coord(j);

		// If the new site is unoccupied we set its probability to A
		if(_grid.find( hash_fun(site) ) == _grid.end())
		{
			f(j) = A;
		// else we set it to epsilon
		}else{
			//std::cout << site.transpose() << " is occupied." << std::endl;
			f(j) = eps;
		}
	}

	if( (f.sum()-1) < 0 )
	{
		//std::cout << "KNOT " << std::endl;
	}

	return f/f.sum();
}

Eigen::VectorXd Chain::fractal_globule()
{
	double A = 10000;
	double epsilon = 0.0001;
	// Porbability Distribution of sites
 	Eigen::VectorXd f = Eigen::VectorXd::Zero(NR_OF_DIRECTIONS);

	Eigen::VectorXd pos = _chain.back().pos;
	// For the six different directions (up down all around)
	int occupied = 0;
	for(int j = 0; j<NR_OF_DIRECTIONS; j++)
	{
		// A site next to pos
		Eigen::Vector3d site = pos + int_to_coord(j);

		// If the new site is unoccupied we see how many neigburs it has
		if(_grid.find( hash_fun(site) ) == _grid.end())
		{
			int neighbours = 0;
			for(int k =0; k<NR_OF_DIRECTIONS; k++)
			{
				// if the neighbour is occupied
				Eigen::Vector3d site_neighbour = site + int_to_coord(k);
				if( _grid.find( hash_fun( site_neighbour)  ) != _grid.end() )
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

	if( (f.sum()-1) < 0 )
	{
	//	std::cout << "KNOT " << std::endl;
	}

	// If we are in a place where all neighbouring places are occupied
	// we have a knot
	/*
	if(occupied == NR_OF_DIRECTIONS)
	{
		_knots_check++;	
		if(!_knots.empty())
		{
			knot lk = _knots.back();
			if( (lk.start+lk.len) == l.nr )
			{
				_knots[_knots.size()-1].len++;
			}else{
				knot k = {l.nr, 1};
				_knots.push_back(k);
			}
		}else{
				knot k = {l.nr, 1};
				_knots.push_back(k);
		}
	}
	*/
	return f/f.sum();
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

long long Chain::hash_fun(Eigen::Vector3d x)
{
	// Move all numbers in x to fall between 0 and 2*max_grid_size
	long long m1 = MAX_GRID_SIZE;
	long long m2 = m1*m1;
	long long m3 = m1*m1*m1;

	//std::cout << m3 << std::endl;

	long long p1 = x(0);
	long long p2 = x(1)*m1;
	long long p3 = x(2)*m2;
	//std::cout << p1 << ", " << p2 << ", " << p3 << std::endl;
	if( (p1 >=   m1) ||
		(p1 <=  -m1) ||
		(p2 >=   m2) ||
		(p2 <=  -m2) ||
		(p3 >=   m3) ||
		(p3 <=  -m3)){
			std::cerr << "Int overflow in Chain.cpp::hash_fun "<< x.transpose() << std::endl;
			std::cerr <<  p1 << " " << p2 <<"  "<< p3 << std::endl;
			std::cerr <<  m1 << " " << m2 <<"  "<< m3 << std::endl;
			exit(1);
		}
	return p1 + p2 + p3;
}
		
Eigen::ArrayXXd Chain::as_array()
{
	return as_array(0,_N);
}
Eigen::ArrayXXd Chain::as_array(int start, int end)
{
	Eigen::ArrayXXd ret = Eigen::ArrayXXd::Zero(end-start,3);
	int i = 0;
	for( int idx = start;  idx < end; idx++ )
	{
		Eigen::Vector3d v = _chain[idx].pos;
		ret(i,0) = v(0);
		ret(i,1) = v(1);
		ret(i,2) = v(2);
		i++;
	}

	return ret;
}

std::ostream& operator<<(std::ostream& os, const Chain& c)
{
	for( auto ptr = c._chain.begin(); ptr!= c._chain.end(); ptr++)
	{
		Eigen::Vector3d v = ptr->pos;
		os <<std::setw(PRINT_SPACING)<< v(0)<< " " <<std::setw(PRINT_SPACING) << v(1) <<" " << std::setw(PRINT_SPACING) << v(2) << " " << std::endl;	
	}
	return os;
}

Eigen::VectorXd Chain::PDF_to_CDF(Eigen::VectorXd f)
{
	int len = f.size();
	Eigen::VectorXd F = Eigen::VectorXd::Zero(len+1);

	for(int i=0; i<len; i++)
	{
		F(i+1) = f.segment(0,i+1).sum();
	}
	return F;
}


double Chain::get_mean_squared_distance()
{
	return get_mean_squared_distance(0,_N);
}

double Chain::Rg()
{
	return Rg(0,_N);
}


Eigen::Vector3d Chain::get_CM()
{
	return get_CM(0,_N);
}

double Chain::get_mean_squared_distance(int start, int end)
{
	Eigen::Vector3d len = _chain[end-1].pos - _chain[start].pos;
	return len.norm();
}

Eigen::Vector2d Chain::get_binned_mean_square_distance(int start, int end)
{
	Eigen::ArrayXd len = Eigen::ArrayXd::Zero(end-start);
	int j = 0;
	for(int i = start; i<end; i++)
	{
		len(j) +=get_mean_squared_distance(0,i);
		j++;
	}
	double mean = len.sum()/( (double) len.size() );
	double var = 0;
	for(int i = 0; i<len.size(); i++)
	{
		var += pow(len(i) - mean,2);
	}
	var = var/len.size();

	return Eigen::Vector2d(mean,var);
}

double Chain::Rg(int start, int end)
{
	Eigen::Vector3d mean = get_CM(start, end);
	double var = 0;
	for(int i = start; i<end; i++)
	{
		Eigen::Vector3d v = _chain[i].pos - mean;
		var += v.transpose() * v;
		
	}
	return sqrt(var/((double) (end-start)));
}

Eigen::Vector3d Chain::get_CM(int start, int end)
{
	Eigen::Vector3d cm(0,0,0);
	for(int i = start; i<end; i++)
	{
		cm += _chain[i].pos;
	}
	cm = cm/((double) (end-start));
	return cm;
}
