#include "Particle.hpp"

#include <iostream>
Particle::Particle(double rad, Eigen::Array3d pos, CollisionGrid* gr)
{
	grid = gr;

	x = pos.matrix();
	r = rad;
	S = Sphere(x, rad);
	v = Eigen::Vector3d(1,0,0);
	first_step_taken = false;

	traj = std::vector<Eigen::VectorXd>();
}

Particle::~Particle()
{

}

Eigen::Array3d Particle::get_position()
{
	return x;
}

Eigen::Array3d Particle::get_velocity()
{
	return v;
}

void Particle::update(double dt, Eigen::Array3d a)
{
	// Gonna implement som basic newtonian bouncing just to se that it works
	// Using leapfrog
	double h = dt;

	if(!first_step_taken)
	{
		first_step_taken = true;
		h = dt/2;
	}

	Eigen::Vector3d vp = v + h*a.matrix();
	Eigen::Vector3d xp = x + dt*v;

	Plane P = Plane(Eigen::Array3d(0,0,0), Eigen::Array3d(0,1,0));
	
	if(P.intersects(&S))
	{
//		std::cout << x.transpose() << std::endl;
		vp(1) = -v(1);
		xp(1) = r;
	}
	
	v = vp;
	x = xp;
	S = Sphere(x.array(),r);

	Eigen::VectorXd tmp = Eigen::VectorXd::Zero(6);
	tmp.segment(0,3) = x;
	tmp.segment(3,3) = v;
	traj.push_back(tmp);
}

std::ostream& operator<<(std::ostream& os, const Particle& p)
{
	for(auto it = p.traj.begin(); it != p.traj.end(); it++)
	{
		os << it->transpose() << std::endl;
	}
	return os;
}
