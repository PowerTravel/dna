#ifndef COLLISION_GEOMETRY_HPP
#define COLLISION_GEOMETRY_HPP

#include <Eigen/Dense>
#include <vector>
#ifndef COLLISION_GEOMETRY_TYPEDEF
#define COLLISION_GEOMETRY_TYPEDEF
class CollisionGeometry;
typedef CollisionGeometry collGeom;
#endif  // COLLGEOM_TYPEDEF
class Sphere;

class CollisionGeometry
{
	public:
	/*
		struct axis_aligned{
			// Center
			Eigen::Array3d cm;
			// Span in x,y,z dir
			double x,y,z;
			
			std::vector<Eigen::Array3d> get_corners();
		};	
	*/
		CollisionGeometry();
		virtual ~CollisionGeometry();

		virtual bool intersects(Sphere* s) = 0;
		virtual Eigen::ArrayXd get_span() = 0;
//		virtual axis_aligned get_axis_aligned() = 0;
	protected:
		
};

#endif // COLLISION_GEOMETRY_HPP
