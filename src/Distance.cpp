#include "Distance.hpp"

Distance::Distance(std::map<std::string, std::string> sm) : Simulation(sm)
{
	_simulation_type = val_map.at("distance");
	init_simulaition_parameters(sm);
}

void Distance::init_simulaition_parameters(std::map<std::string, std::string> sm)
{
	if( sm.find("SIZE") != sm.end() )
	{
		_size  = text_to_int(sm["SIZE"]);
	}else{
		_valid = false;
	}

/*
	if( sm.find("STRIDES") != sm.end() )
	{
		_strides  = text_to_int(sm["STRIDES"]);
	}else{
		_valid = false;
	}

	if( sm.find("EXP") != sm.end() )
	{
		_exp  = text_to_bool(sm["EXP"]);
	}else{
		_valid = false;
	}

	if( sm.find("SAMPLES") != sm.end() )
	{
		_samples  = text_to_int(sm["SAMPLES"]);
	}else{
		_valid = false;
	}
*/
}

void Distance::apply()
{

	_c->build(_size);

	CollisionGrid cg = CollisionGrid();
	cg.set_chain(_c);

	

	//std::cerr << "Simulation Distance is not implemented." << std::endl;
	//std::cerr << this << std::endl;
}

void Distance::print(std::ostream& os)
{
	os <<"Simulation = Distance" << std::endl;
	if(_valid)
	{
		os <<"Out File   = " << _outfile << std::endl;
		os <<"Chain Size = " << _size<< std::endl;
		os <<"Use Weight = ";
		if(weight)
		{
			os << "true" << std::endl;
		}else{
			os << "false" << std::endl;
		}
		os <<"Allow selfintersection = ";
		if(selfint)
		{
			os << "true" << std::endl;
		}else{
			os << "false" << std::endl;
		}
		os <<"Allow Selfintersection = " << _size<< std::endl;
//		os <<"Nr Strides = " << _strides << std::endl;
//		os <<"Samples    = " << _samples;
//		os <<"Exponential= " << _exp << std::endl;
	}else{
		os << "Simulation failed to load.";
	}
};
