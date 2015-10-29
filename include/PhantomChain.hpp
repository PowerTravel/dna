#ifndef PHANTOM_CHAIN_HPP
#define PHANTOM_CHAIN_HPP

#include "Chain.hpp"

class PhantomChain : public Chain{
	
	public:
		PhantomChain();
		virtual ~PhantomChain();

		void build(int N);

};

#endif
