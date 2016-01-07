#ifndef PLANE_HPP
#define PLANE_HPP

#include "CollisionGeometry.hpp"

class Plane: public CollisionGeometry{

	public:	

		friend class Sphere;

		Plane(Eigen::Vector3d xp, Eigen::Vector3d np);
		virtual ~Plane();

		bool intersects(Sphere* s, coll_struct& cs);
		double line_intersection_point(Eigen::ArrayXd x, Eigen::ArrayXd v);
		Eigen::ArrayXd get_span();

	private:

		Eigen::Vector3d n;
		Eigen::Vector3d x;

};

#endif
