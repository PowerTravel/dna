#ifndef PLANE_HPP
#define PLANE_HPP

#include "CollisionGeometry.hpp"

class Plane: public CollisionGeometry{

	public:	

		friend class Sphere;

		Plane(Eigen::Vector3d xp, Eigen::Vector3d np);
		virtual ~Plane();

		bool intersects(Sphere* s, coll_struct& cs);
		double line_intersection_point(Vec3d x, Vec3d v);
		Eigen::ArrayXd get_span();
		virtual Vec3d get_center();

		std::string text_type();
	private:

		Eigen::Vector3d n;
		Eigen::Vector3d x;

};

#endif
