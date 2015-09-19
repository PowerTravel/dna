#ifndef SPRING_HPP
#define SPRING_HPP

class Spring{
	public:
		Spring(double a= 1.0, double k = 1.0, double xi = 0.1);
		virtual ~Spring();

		double getForce( double dx, double dv);

	private:

		double _k  = 1.0; // Spring Constant
		double _xi = 0.1; // Damping Coefficient
		double _a  = 1.0; // Rest lenght;

};

#endif // SPRING_HPP
