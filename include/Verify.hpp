#ifndef VERIFY_HPP
#define VERIFY_HPP

#include "Simulation.hpp" 

/*
	Valid parameters
	"NR_STRIDES" = UNSIGNED_INTEGER_TYPE;
	"STRIDE_LEN" = UNSIGNED_INTEGER_TYPE;
	"GROWTH" = DOUBLE_TYPE;
	"SAMPLES" = UNSIGNED_INTEGER_TYPE;
*/

class Verify : public  Simulation{
	
	public:
		Verify();
		Verify(std::map<std::string, std::string> sm);
		virtual ~Verify();

		virtual void print(std::ostream& os) const{ os << "This is the derived class.";};
		
	private:
		int _nr_strides;
		int _stride_len;
		double _growth;
		int _samples;
		

};

#endif //  VERIFY_HPP
