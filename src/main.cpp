#include <iostream>
#include <Eigen/Dense>

#include "Chain.hpp"

// Vice Värd  070 6699467

int main(int argc, char* argv[])
{

	/* Generate a chain or load chain configuration from file */
	Chain polymer = Chain(2);

	Eigen::VectorXd p = polymer.getPos();

	polymer.printChain();
	polymer.update(0.1);
	polymer.update(0.1);
	polymer.update(0.1);
	polymer.printChain();

	for(int i = 0; i<p.size(); i++)
	{
		//std::cout << p(i) << std::endl;
	}

	/* Simulate the chain */


	/* Print data to file */

	std::cout << "All OK!" << std::endl;
	return 0;
}
