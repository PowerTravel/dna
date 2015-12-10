#include "Particle.hpp"

#include <iostream>
Particle::Particle(double rad, Eigen::Array3d pos, CollisionGrid* gr)
{
	grid = gr;

	x = pos.matrix();
	r = rad;
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

	Plane P = Plane(Eigen::Vector3d(0,0,0), Eigen::Vector3d(0,1,0));
	Sphere S = Sphere(xp, r);
	if(P.intersects(&S))
	{	
		Eigen::Vector3d pn = P.getPlaneNormal();	
		// Hitta kollsionspunkten som är 
		// Porjicera en linje mellan planet och sfären för att hitta
		// first contact punkten
		Eigen::Vector3d contact_point = x - r * pn;
		double dt_t = P.line_intersection_point(contact_point, v);
		Eigen::Vector3d x_c = x +  dt_t * v;

		// Komposantuppdela hastigheten runt plane normal
		double len = v.transpose() * pn;
		Eigen::Vector3d v_paralell  = len * pn;
		Eigen::Vector3d v_antiparalell  = v - v_paralell;
	
		vp = v_antiparalell - v_paralell;
		xp = x_c +(dt-dt_t) * vp;
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
