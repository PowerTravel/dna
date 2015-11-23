#ifndef PFLOAT_HPP
#define PFLOAT_HPP
#include <gmp.h>
#include <iostream> 
// Wrapper class for gmp
class PFloat{

	public:
		PFloat()
		{
			mpf_init(v);
		};

		PFloat(float val)
		{
			mpf_init(v);
			mpf_set_d(v,val);
		}

		virtual ~PFloat(){};

		//friend PFloat operator=(const PFloat& a);
		PFloat& operator=(const PFloat& a)
		{
			mpf_init(v);
			mpf_set(v,a.v);
			return *this;
		};

		PFloat& operator=(const float& a)
		{
			mpf_init(v);
			mpf_set_d(v,a);
			return *this;
		};

		friend PFloat operator+(const PFloat& a, const PFloat& b)
		{
			PFloat ret = PFloat();
			mpf_add(ret.v, a.v, b.v);
			return ret;
		};

		friend PFloat operator*(const PFloat& a, const PFloat&b)
		{
			PFloat ret = PFloat();
			mpf_mul(ret.v,a.v,b.v);
			return ret;
		};
		
		double as_float()
		{
			double ret = mpf_get_d(v);
			if( (ret == 0) && ( *this != 0 ))
			{
				this->print();
				std::cerr << "PFloat has ben truncated in PFloat::as_float()" << std::endl;
				exit(1);
			}

			return ret;
		};

		static PFloat div(PFloat& a,PFloat& b)
		{
			PFloat ret = PFloat();
			mpf_div(ret.v,a.v,b.v);
			return ret;
		};

		friend PFloat operator/(const PFloat& a, const PFloat& b)
		{
			PFloat ret = PFloat();
			mpf_div(ret.v,a.v,b.v);
			return ret;
		};
		
		friend bool operator==(const PFloat& a, const PFloat& b)
		{
		
			if( mpf_cmp(a.v, b.v) == 0)
			{
				return true;
			}else{
				return false;
			}
		};
		
		friend bool operator!=(const PFloat& a, const PFloat& b)
		{
			return !(a == b);
		};


		void print()
		{
			mpf_out_str(stdout, 10, 5, v);
		};

	private:
		
		mpf_t v;

};

#endif //  PFLOAT_HPP
