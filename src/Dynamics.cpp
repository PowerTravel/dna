#include "Dynamics.hpp"

Dynamics::Dynamics()
{
	x = Eigen::Vector3d::Zero();
	v = Eigen::Vector3d::Zero();
}

Dynamics::Dynamics(Eigen::Vector3d xp, Eigen::Vector3d vp)
{
	x = xp; 
	v = vp;
}

Dynamics::~Dynamics()
{

}
Eigen::Vector3d Dynamics::getPos()
{
	return x;
}
Eigen::Vector3d Dynamics::getVel()
{
	return v;
}
