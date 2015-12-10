#ifndef PLANE_HPP
#define PLANE_HPP

#include "CollisionGeometry.hpp"

class Plane: public CollisionGeometry{

	public:	
		Plane(Eigen::Array3d xp, Eigen::Array3d np);
		virtual ~Plane();

		bool intersects(Sphere* s);
		bool intersects(Plane* p);
		Eigen::ArrayXd get_span();

		Eigen::ArrayXd getPlaneNormal();
		Eigen::ArrayXd getPoint();

	private:

		Eigen::Array3d n;
		Eigen::Array3d x;

};

#endif
