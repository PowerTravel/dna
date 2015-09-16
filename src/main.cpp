#include <iostream>
#include <Eigen/Dense>

int main(int argc, char* argv[])
{

	Eigen::MatrixXd m(2,2);
	m(0,0) = 1;
	m(0,1) = 2;
	m(1,0) = 3;
	m(1,1) = 4;
	std::cout << m << std::endl;
	
	return 0;
}
