#ifndef SPRING_HPP
#define SPRING_HPP

#include <Eigen/Dense>

class Spring{
	public:
		Spring(double a= 1.0, double k = 1.0);
		virtual ~Spring();

		double getForce( double dx );
		Eigen::Vector3d getForce( Eigen::Vector3d& p1, Spring& s1,Eigen::Vector3d& p2, Spring& s2, Eigen::Vector3d& p3);

	private:

		static double _a; // Rest lenght
		static double _k; // Spring Constant
};

#endif // SPRING_HPP
