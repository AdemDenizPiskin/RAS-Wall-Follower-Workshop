#ifndef ROBOT_H
#define ROBOT_H

namespace ras
{
	class robot
{
	public:
		float sens1_pos;
		float sens2_pos;
		float sens3_pos;
		float sens4_pos;

		
		robot(float sens1,float sens2,float sens3,float sens4);
		robot(float sens1,float sens2,float sens3);
		robot(float sens1,float sens2);
		robot(float sens1);
		robot();
		
		void generate_path(bool put_wall);
		void print_path();
		
		double * read_sens1();
		double * read_sens2();
		double * read_sens3();
		double * read_sens4();
		
		void print_prediction(double *pred_r, double *pred_theta);
		
		
		double MSE_r(double *pred_r);
		double MSE_theta(double *pred_theta);
		
		void print_wall_pos();
		
		void stop(int t_s);
		
		private:
			double r[1000];
			double theta[1000];
			int wall_pos;
	
};

}



#endif
