#ifndef COLLISION_GEOMETRY_HPP
#define COLLISION_GEOMETRY_HPP

#include <Eigen/Dense>
#include <vector>
#include <iostream>

class Sphere;
class Plane;
class Cylinder;

class CollisionGeometry
{
	public:
		struct coll_struct{
			double p; // Penetration_depth
			Eigen::Vector3d n; // Collision-plane-normal
		};
	
		virtual bool intersects(Cylinder* s, coll_struct& cs) = 0;
		virtual bool intersects(Sphere* s, coll_struct& cs) = 0;
		virtual bool intersects(Plane* p, coll_struct& cs) = 0;
		virtual double line_intersection_point(Eigen::ArrayXd x, Eigen::ArrayXd v) = 0;
		virtual Eigen::ArrayXd get_span() = 0;
		
};

#endif // COLLISION_GEOMETRY_HPP
