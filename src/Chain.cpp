#include "Chain.hpp"
#include <iostream>
#include <cmath>
#include <iomanip>
#include <ctime>

std::default_random_engine Chain::_generator = std::default_random_engine(time(NULL));

//std::map< int, std::string > Chain::_int_dir = Chain::DirMapConstructor::int_to_dir_map();
//std::map< std::string, int > Chain::_dir_int = Chain::DirMapConstructor::dir_to_int_map();

Chain::Chain()
{
	_verbose = false;
	_redo = 0;
}
/*
Chain::Chain(int seed)
{
	_redo = 0;
	_verbose = false;
	_generator = std::default_random_engine(seed);
}
*/

Chain::~Chain()
{

}


void Chain::update(double dt)
{

}

void Chain::generateGlobule(int N)
{
	_globule.clear();
	_knots.clear();


	_N = N;
	_knots_check = 0;
	if(_verbose){
	std::cout <<  "["<< std::setw(3) << 0 << "%] "
				<<"Generating Globule with " << N << " links:" << std::flush;
	}

	// Make sure not to reallocate the _globule vector while g_map is active
	std::map<int, int >  g_map = std::map<int, int >();
	_globule = std::vector< link >(N); 
	_knots = std::vector< knot >(); 
	
	// Start position
	_globule[0] = link(0, Eigen::Vector3d(0,0,0));

	g_map[hash_fun(_globule[0].pos)] = 0; // Start link has the unique identifier 0

	int dots = 0;
	double percent = 0;
	for(int i = 1; i<N; i++)
	{
		_n = i;

		if( (i-1 == floor(N*percent)) && _verbose )
		{
			percent += 0.01;
			std::cout << "\r"<< "["<< std::setw(3) <<  percent*100 << "%] "
						<<"Generating Globule with " << N << " links:" << std::flush;
		}
	
		Eigen::Vector3d pos =_globule[i-1].pos +  getNextStep(g_map);
		_globule[i] = link(i, pos);

		g_map[hash_fun(_globule[i].pos)] = coord_to_int(_globule[i].pos - _globule[i-1].pos);
	}

	if(_verbose)
	{
		std::cout << std::endl;
		std::cout << "RN = " << sqrt(_globule[N-1].pos.norm()) << "  N = "  << N << " N^(1/3) = " <<pow(N,1.0/3.0) << std::endl;
		std::cout << _knots.size() << " knots with a combined length of " << _knots_check << " links" << std::endl;
		std::cout << "That is " << ((double)_knots_check)/((double) N)*100 << " percent of total nr of links." << std::endl;
	}
	double knot_percent = ((double)_knots_check)/((double) N)*100;
	if(knot_percent > 0.1)
	{
		_redo++;
	//	generateGlobule(N);
	}
	
}

//Chain::link Chain::getNextLink(std::map<int,int>& m)
Eigen::Vector3d Chain::getNextStep(std::map<int,int>& m)
{
	// Find the probability distribution for the different directions
	Eigen::VectorXd f = get_stepping_PDF(m);
	//std::cout << f.transpose() << std::endl;

	// Convert it to a Cumulative Distribution function
	Eigen::VectorXd F = PDF_to_CDF(f);

	// Choose the next step acording to F
	Eigen::Vector3d ns = nextStep(F);
	
	return ns;
}

Eigen::Vector3d Chain::nextStep(Eigen::VectorXd F)
{
	std::uniform_real_distribution<double> distribution(0.0,1.0);

	double rand_nr = distribution(_generator);
	int i = 0;

	// What happens if rand_nr is exactly 0?
	while( true  )
	{
		if((rand_nr > F(i)) && ( rand_nr <= F(i+1) ))
		{
			break;
		}
		i++;	
	}
	return int_to_coord(i);
}

Eigen::VectorXd Chain::get_stepping_PDF(std::map<int,int>& m)
{
	double A = 10000;
	double epsilon = 0.0001;
	// Porbability Distribution of sites
 	Eigen::VectorXd f = Eigen::VectorXd::Zero(NR_OF_DIRECTIONS);

	link l = _globule[_n-1];
	// For the six different directions (up down all around)
	int occupied = 0;
	for(int j = 0; j<NR_OF_DIRECTIONS; j++)
	{
		// A site next to pos
		Eigen::Vector3d site = l.pos + int_to_coord(j);

		// If the new site is unoccupied we see how many neigburs it has
		if(m.find( hash_fun(site) ) == m.end())
		{
			int neighbours = 0;
			for(int k =0; k<NR_OF_DIRECTIONS; k++)
			{
				// if the neighbour is occupied
				if( m.find( hash_fun(site + int_to_coord(k)) ) != m.end() )
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
	
	return f/f.sum();
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
		
std::ostream& operator<<(std::ostream& os, const Chain& c)
{
	for(int i = 0; i<c._N; i++)
	{
		Eigen::Vector3d v = c._globule[i].pos;
		os <<std::setw(PRINT_SPACING)<< v(0)<< " " <<std::setw(PRINT_SPACING) << v(1) <<" " << std::setw(PRINT_SPACING) << v(2) << " " << std::endl;	
	}
	return os;
}

