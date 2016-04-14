#ifndef VOL_BOX_HPP
#define VOL_BOX_HPP

class VolBox{
	
	// Six funciton, one for each side.
	struct side_density{
		int wMin;
		int wMax;
		int hMin;
		int hMax;
		int Offset;

		int CO;
		int EO;
		int BO;
		
		int Vol;

		double Density;
		
		bool dirty;
	};

	void shrink_side(int i);

	private:
		int x_min;
		int x_max;
		int y_min;
		int y_max;
		int z_min;
		int z_max;
		
		double x_min_dens;
		double x_max_dens;
		double y_min_dens;
		double y_max_dens;
		double z_min_dens;
		double z_max_dens;
};

void set_side(side_density s[])
{
	for(int sideIdx = 0; sideIdx<6; sideIdx++;)
	switch(sideIdx)
	{
		case 0:
		{
			
			x_min++;
		}break;
		case 1:
		{
			x_max--:
		}break;
		case 2:
		{
			y_min++;
		}break;
		case 3:
		{
			y_max--
		}break;
		case 4:
		{
			z_min++;
		}break;
		{
			z_max--;
		}
		default:
		{
			
		}break;
	};	
}

void shrink_side(int sideIdx)
{
	switch(sideIdx)
	{
		case 0:
		{
			x_min++;
		}break;
		case 1:
		{
			x_max--:
		}break;
		case 2:
		{
			y_min++;
		}break;
		case 3:
		{
			y_max--
		}break;
		case 4:
		{
			z_min++;
		}break;
		{
			z_max--;
		}
		default:
		{
			
		}break;
	};
}

#endif