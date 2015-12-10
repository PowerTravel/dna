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

	if( sm.find("BOX_SIZE") != sm.end() )
	{
		_box_size  = text_to_double(sm["BOX_SIZE"]);
	}else{
		_valid = false;
	}

	if( sm.find("RADIUS") != sm.end() )
	{
		_rad  = text_to_double(sm["RADIUS"]);
	}else{
		_valid = false;
	}
}

void Distance::apply()
{
	print_pre_info();
	if(!_valid)
	{
		std::cerr << "Verify not valid. Exiting" << std::endl;
		return;
	}
	std::cout << this << std::endl;
	_c->set_radius(_rad);
	_c->build(_size);
	CollisionGrid cg = CollisionGrid(_box_size);
	cg.set_up(_c);

	Particle p = Particle(0.5, Eigen::Array3d(0,4,0), &cg);
	for(int i = 0; i < 20; i++)
	{
		p.update(0.1,Eigen::Array3d(0,-9.82,0) );
	}
	
	std::ofstream file;
	file.open(_outfile, std::fstream::out | std::fstream::trunc);
	if(file.is_open()){
		file << p << std::endl;
	}else{
		std::cerr << "Failed to open " << std::string(_outfile) << std::endl;
	}

	file.close();

	print_post_info();
	
}

void Distance::print(std::ostream& os)
{
	os <<"Simulation = Distance" << std::endl;
	if(_valid)
	{
		os <<"Out File   = " << _outfile << std::endl;
		os <<"Chain Size = " << _size<< std::endl;
		os <<"Use Weight = ";
		if(_weight)
		{
			os << "true" << std::endl;
		}else{
			os << "false" << std::endl;
		}
		os <<"Allow selfintersection = ";
		if(_selfint)
		{
			os << "true" << std::endl;
		}else{
			os << "false" << std::endl;
		}
		os <<"Box_Size of collision grid = " << _box_size << std::endl;
		os <<"Radius of chain links  = " << _rad << std::endl;
	}else{
		os << "Simulation failed to load.";
	}
};
