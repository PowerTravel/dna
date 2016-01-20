#include "Verify.hpp"
#include <iomanip>

#include "Statistics.hpp"
typedef Statistics stat;


Verify::Verify()
{

}

Verify::Verify(std::map<std::string, std::string> sm) : Simulation(sm)
{
	_simulation_type = val_map.at("verify");
	init_simulaition_parameters(sm);
	set_chain_type();
	
	init_plotting_parameters();
}
Verify::~Verify()
{

}

void Verify::init_simulaition_parameters(std::map<std::string, std::string> sm)
{
	if( sm.find("SIZE") != sm.end() )
	{
		_size  = text_to_int(sm["SIZE"]);
	}else{
		_valid = false;
	}

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
}

void Verify::set_chain_type()
{
	switch(_chain_type)
	{
		case VAL_BIT_PHANTOM: 
			_theoretical_slope = PHANTOM_SLOPE;
			_theoretical_Rg_slope = PHANTOM_SLOPE;
			break;

		case VAL_BIT_SAW:
			_theoretical_slope = SAW_SLOPE;
			_theoretical_Rg_slope = SAW_SLOPE;
			break;

		case VAL_BIT_FG:
			_theoretical_slope = FG_SLOPE;
			_theoretical_Rg_slope = FG_SLOPE;
			break;
	}
}

void Verify::set_plot_points()
{
	Eigen::ArrayXd n = Eigen::ArrayXd::Zero(_strides+1);

	double start_point = double( START_POINT );
	double N = (double) _size;
	double steps =(double) _strides;

	double k = log(N / start_point) / steps; // exponential
	
	double dn = (N - start_point) / steps; // linear

	for(int i = 0; i<steps+1; i++){
		if(_exp)
		{
			n(i) = start_point * exp(k*i);
		}else{
			n(i) = i * dn + start_point;	
		}
	}
		
	nr_links = n;
	link_mean = (n.segment(0,steps) + n.segment(1,steps)) / 2;
}


void Verify::set_theoretical_values()
{
	switch(_chain_type)
	{
		case VAL_BIT_PHANTOM: 
			R_theo = PHANTOM_R_FF * link_mean.pow(PHANTOM_SLOPE);
			Rg_theo = PHANTOM_RG_FF * link_mean.pow(PHANTOM_SLOPE);
			break;

		case VAL_BIT_SAW:
			R_theo = SAW_R_FF * link_mean.pow(SAW_SLOPE);
			Rg_theo = SAW_RG_FF * link_mean.pow(SAW_SLOPE); // Forefactor from polymer
														// textbook p 40 numerical
			break;

		case VAL_BIT_FG:
			R_theo = FG_R_FF * link_mean.pow(FG_SLOPE);
			Rg_theo = FG_RG_FF * link_mean.pow(FG_SLOPE); // Forefactor from polymer
			break;
	}
}

void Verify::init_plotting_parameters()
{
	double interval = 1; // How big our update step should be,
						 // 1== update every time
	_increment = 1.f/((_strides-1)*(_samples-1));
	_plot_interval = interval * _increment;
	_percent = 0.0;
}

void Verify::print(std::ostream& os)
{
	os <<"Simulation = Verify" << std::endl;
	if(_valid)
	{
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
		os <<"Out File   = " << _outfile << std::endl;
		os <<"Chain Size = " << _size<< std::endl;
		os <<"Nr Strides = " << _strides << std::endl;
		os <<"Samples    = " << _samples;
		os <<"Exponential= " << _exp << std::endl;
	}else{
		os << "Simulation failed to load.";
	}
};

void Verify::apply()
{
	print_pre_info();
	if(!_valid)
	{
		std::cerr << "Verify not valid. Exiting" << std::endl;
		return;
	}

	int steps = _strides;
	int samples = _samples;
	int N = _size;

	set_plot_points();
	set_theoretical_values();

	Eigen::ArrayXXd binned_chain = Eigen::ArrayXXd::Zero(steps,3*samples);
	std::vector< std::vector<PFloat> > w = std::vector< std::vector<PFloat> >(steps);
	for(int i = 0; i < steps; i++)
	{
		w[i] = std::vector<PFloat>(samples);	
	}
	Eigen::ArrayXXd Rg_tmp = Eigen::ArrayXXd::Zero(steps,samples);

	for(int i = 0; i < samples; i++)
	{	
		if(_c != NULL)
		{
			_c->build(N);
		}else{
			std::cerr << "ERROR: VERIFY::APPLY Chain IS NULL" << std::endl;
			return;
		}

		// Vector containing the weight for each individual link
		Eigen::VectorXd w_tmp = _c->weights();
		// bin values
		for(int j = 0; j<steps; j++)
		{
			Eigen::ArrayXXd sub_chain = _c->as_array( nr_links(j), nr_links(j+1)-nr_links(j) ).transpose();

			binned_chain(j,3*i+0) = sub_chain.col(0).mean();
			binned_chain(j,3*i+1) = sub_chain.col(1).mean();
			binned_chain(j,3*i+2) = sub_chain.col(2).mean();

			Rg_tmp(j,i) = _c->Rg(0,floor(link_mean(j)));

			w[j][i] = mult_weights( w_tmp.segment(0, nr_links(j+1)) );
		}

		if(verbose)
		{
			write_to_terminal(N,i,steps);
		}
	}
	Eigen::ArrayXXd R_tmp = Eigen::ArrayXXd::Zero(steps,samples);

	R 		= Eigen::ArrayXd::Zero(steps);
	R_var 	= Eigen::ArrayXd::Zero(steps);

	Rg 		= Eigen::ArrayXd::Zero(steps);
	Rg_var 	= Eigen::ArrayXd::Zero(steps);
	
	for(int i = 0; i < steps; i++)
	{
		for(int j = 0; j < samples; j++)
		{
			Eigen::Vector3d pos  = binned_chain.block(i,3*j,1,3).transpose();
			R_tmp(i,j) = pos.norm();
		}
	}

	for(int i = 0; i<steps; i++)
	{
		Eigen::Vector2d mv = stat::get_mean_and_variance(R_tmp.row(i), w[i]);
		R(i) = mv(0);
		R_var(i) = mv(1);
		
		mv = stat::get_mean_and_variance(Rg_tmp.row(i), w[i]);
		Rg(i) = mv(0);
		//std::cout << Rg_tmp.row(i).mean() <<"  " << Rg(i) <<"  "<< Rg_theo(i) << std::endl;
		Rg_var(i) = mv(1);
	}

	write_to_file();
	print_post_info();
}

void Verify::write_to_file()
{
	int idx = nr_links.size()-1;
	Eigen::MatrixXd result = Eigen::MatrixXd::Zero(idx, 23);

	result.block(0,0,idx,1) = link_mean;
	result.block(0,1,idx,1) = R;
	result.block(0,2,idx,1) = R_var;
	result.block(0,3,idx,1) = R_theo;
	
	result.block(0,4,idx,1) = Rg;
	result.block(0,5,idx,1) = Rg_var;
	result.block(0,6,idx,1) = Rg_theo;

	result.block(0,21,idx,1) = link_mean;
	std::ofstream file;
	file.open(_outfile, std::fstream::out | std::fstream::trunc);
	if(file.is_open()){
		file << result << std::endl;
	}else{
		std::cerr << "Failed to open " << std::string(_outfile) << std::endl;
	}
	file.close();
}

void Verify::write_to_terminal(int N, int i, int j)
{
	double p = ( ( double) i*nr_links(nr_links.size()-1) + j) / ((double) _samples*nr_links(nr_links.size()-1)) * 100.f;
		std::cout << "\r"<< "["<< std::setw(5)  << std::setprecision(1)<< p << std::fixed << "%] "
			<<"Generating " << _samples <<" " <<dictionary.at(_chain_type) <<"s with " << N << " links. " <<  std::flush;
}

PFloat Verify::mult_weights(Eigen::ArrayXd w)
{
	PFloat ret = PFloat(w[0]);

	for(int i = 1; i < w.size(); i++)
	{
		PFloat tmp = PFloat(w(i));
		ret = ret*tmp;
	}
	return ret;
}
