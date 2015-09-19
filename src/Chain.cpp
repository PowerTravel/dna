#include "Chain.hpp"
#include <iostream>

Chain::Chain(int N )
{
	/* Variables */

	// For springs
	double k  = 1.0; // Spring Constant
	double xi = 0.1; // Damping Coefficient
	double a  = 1.0; // Rest lenght;
	
	// For Links
	double rad = a/2;

	_links = std::vector<Sphere>(N);
	for(int i = 0; i<N; i++)
	{
		Eigen::Vector3d x(i*(a+0.1), 0, 0);
		Eigen::Vector3d v(0, 0, 0);
		_links[i] = Sphere(x,v,a);
	}

	_spring = std::vector<Spring>(N-1);
	for(int i = 0; i<N-1; i++)
	{
		_spring[i] = Spring(a);
	}

	_N = N;
}

Chain::~Chain()
{

}

void Chain::update(double dt)
{
	Eigen::VectorXd F = Eigen::VectorXd(_N-1);
	Eigen::Vector3d direction = Eigen::VectorXd(3*(_N-1));
	
	// Get the forces between the particles
	for(int i = 0; i<_N-1; i++)
	{
		Eigen::Vector3d dx = _links[i+1].getPos() - _links[i].getPos();
		Eigen::Vector3d dv = _links[i+1].getVel() - _links[i].getVel();
		//F(i) =_spring[i].getForce( dx.norm(), dv.norm() );
		
		F(i) =_spring[i].getSpeed( dx.norm());
		direction.segment(3*i,3) = dx/dx.norm();
	}
	
		
	for(int i = 0; i<_N-1; i++)
	{
		if(	i == 0  )
		{
			std::cout << "Continue at Chain.cpp:59 calculating forces" << std::endl;
			Eigen::Vector3d dx = dt*F(i)*direction.segment(3*i,3);
			_links[0].move(dx);
		}
		
	}

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
