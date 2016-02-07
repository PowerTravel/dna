#ifndef COLLISION_GEOMETRY_HPP
#define COLLISION_GEOMETRY_HPP

#include "EigenLibs.hpp"

#include <vector>
#include <iostream>
#include <memory>

#ifndef COLLISIONGEOMETRY_PTR
class CollisionGeometry;
typedef std::shared_ptr<CollisionGeometry> cg_ptr;
#endif

class Sphere;
class Plane;
class Cylinder;
class Capsule;

class CollisionGeometry
{
	public:
		CollisionGeometry();
		virtual ~CollisionGeometry();

		struct coll_struct{
			double p; // Penetration_depth
			Eigen::Vector3d n; // Collision-plane-normal
		};
	
		virtual bool intersects(Cylinder* s, coll_struct& cs);
		virtual bool intersects(Sphere* s, coll_struct& cs);
		virtual bool intersects(Plane* p, coll_struct& cs);
		virtual double line_intersection_point(Vec3d x, Vec3d v);
		virtual Eigen::ArrayXd get_span() = 0;
		virtual Vec3d get_center() = 0;
		virtual std::string text_type() = 0;
		int get_id();
	private:
		static int c_id;
		int id;
};

#endif // COLLISION_GEOMETRY_HPP
