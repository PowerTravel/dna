#include <iostream>
#include <Eigen/Dense>

#include "Chain.hpp"

int main(int argc, char* argv[])
{

	/* Generate a chain or load chain configuration from file */
	Chain polymer = Chain();

	Eigen::VectorXd p = polymer.getPos();

	for(int i = 0; i<p.size(); i++)
	{
		//std::cout << p(i) << std::endl;
	}

	/* Simulate the chain */


	/* Print data to file */

	std::cout << "All OK!" << std::endl;
	return 0;
}
