#ifndef DYNAMICS_HPP
#define DYNAMICS_HPP

#include <Eigen/Dense>

class Dynamics{
	
	public:
		Dynamics();
		Dynamics(Eigen::Vector3d xp, Eigen::Vector3d vp);
		virtual ~Dynamics();

		virtual void update() = 0;
		
		Eigen::Vector3d getPos();
		Eigen::Vector3d getVel();

	protected:
		Eigen::Vector3d x;
		Eigen::Vector3d v;
};

#endif
