#include "robot.h"
#include <iostream>
#include <cmath>
#include <sstream>
#include <fstream>
#include <stdlib.h>
#include <random>
#include <chrono>

#define l 		10
#define L 		20
#define dt 		0.01
#define N 		1000
#define v 		20
#define M_PI  	3.14159265358979323846
// dt = 0.01, N = 1000, t = 10, v = 5cm/sec
#define sigma 0.5
#define MAX_R 35
#define CR_P 0.6745
// phi(CR_P) = 0.25


namespace ras
{
	robot::robot(float sens1,float sens2,float sens3,float sens4)
{	
	robot::sens1_pos = sens1;
	robot::sens2_pos = sens2;
	robot::sens3_pos = sens3;
	robot::sens4_pos = sens4;
	
	
	std::cout<<"Robot starting..."<<std::endl;
	std::cout<<"Positions of sensors are given as:"<<std::endl;
	std::cout<<"Sensor 1: "<<sens1_pos<<std::endl;
	std::cout<<"Sensor 2: "<<sens2_pos<<std::endl;
	std::cout<<"Sensor 3: "<<sens3_pos<<std::endl;
	std::cout<<"Sensor 4: "<<sens4_pos<<std::endl; 
}

	robot::robot(float sens1,float sens2,float sens3)
{	
	robot::sens1_pos = sens1;
	robot::sens2_pos = sens2;
	robot::sens3_pos = sens3;
	robot::sens4_pos = -999;
	
	
	std::cout<<"Robot starting..."<<std::endl;
	std::cout<<"Positions of sensors are given as:"<<std::endl;
	std::cout<<"Sensor 1: "<<sens1_pos<<std::endl;
	std::cout<<"Sensor 2: "<<sens2_pos<<std::endl;
	std::cout<<"Sensor 3: "<<sens3_pos<<std::endl;
	std::cout<<"Sensor 4: none"<<std::endl; 
}

	robot::robot(float sens1,float sens2)
{	
	robot::sens1_pos = sens1;
	robot::sens2_pos = sens2;
	robot::sens3_pos = -999;
	robot::sens4_pos = -999;
	
	
	std::cout<<"Robot starting..."<<std::endl;
	std::cout<<"Positions of sensors are given as:"<<std::endl;
	std::cout<<"Sensor 1: "<<sens1_pos<<std::endl;
	std::cout<<"Sensor 2: "<<sens2_pos<<std::endl;
	std::cout<<"Sensor 3: none"<<std::endl;
	std::cout<<"Sensor 4: none"<<std::endl; 
}
	robot::robot(float sens1)
{	
	robot::sens1_pos = sens1;
	robot::sens2_pos = -999;
	robot::sens3_pos = -999;
	robot::sens4_pos = -999;
	
	
	std::cout<<"Robot starting..."<<std::endl;
	std::cout<<"Positions of sensors are given as:"<<std::endl;
	std::cout<<"Sensor 1: "<<sens1_pos<<std::endl;
	std::cout<<"Sensor 2: none"<<std::endl;
	std::cout<<"Sensor 3: none"<<std::endl;
	std::cout<<"Sensor 4: none"<<std::endl; 
}
	robot::robot()
{	
	robot::sens1_pos = -999;
	robot::sens2_pos = -999;
	robot::sens3_pos = -999;
	robot::sens4_pos = -999;
	
	
	std::cout<<"Robot starting..."<<std::endl;
	std::cout<<"Positions of sensors are given as:"<<std::endl;
	std::cout<<"Sensor 1: none"<<std::endl;
	std::cout<<"Sensor 2: none"<<std::endl;
	std::cout<<"Sensor 3: none"<<std::endl;
	std::cout<<"Sensor 4: none"<<std::endl; 
	std::cout<<"Sensor niye takmiyon kardesim"<<std::endl;
}



void robot::generate_path(bool put_wall)
{
	unsigned seed = std::chrono::system_clock::now().time_since_epoch().count();
  	std::default_random_engine generator (seed);
	
  	std::normal_distribution<double> distribution(0.0,1);
	
	std::cout<<"Generating path..."<<std::endl;
	
	int i;
	double mode = distribution(generator);
	
	if(put_wall)
		robot::wall_pos = 850+10*distribution(generator);
	
	else
		robot::wall_pos = 999999;
	
	
	//Damped Oscilator with 3^rd order nonlinear harmonics
	if(mode < (-1)*CR_P)
	{
		
		double A = 0.8*distribution(generator)+3.0;
		double B = 0.2*distribution(generator);
		double f = 2.0*distribution(generator)+10.0;
		double k = fabs(0.4*distribution(generator)+0.2);
		/*
		std::cout<<"Damped oscilation mode"<<std::endl;
		std::cout<<"Parameters:"<<std::endl;
		std::cout<<"A = "<<A<<std::endl;
		std::cout<<"B = "<<B<<std::endl;
		std::cout<<"f = "<<f<<std::endl;
		std::cout<<"k = "<<k<<std::endl;
		*/
		for(i=0;i<N;i++)
		{
		
			if(i==0)
			{
				robot::r[i] = l;
				robot::theta[i]=0;
			}
			else
			{
				robot::r[i] = (B*pow(sin(f*i*dt),3)+A*sin(f*i*dt))*exp(-k*i*dt) + l;
				if(robot::r[i]<0)
					robot::r[i] = 0;
				else if(robot::r[i]>20)
					robot::r[i]=20;
				robot::theta[i] = atan2( (r[i]-r[i-1]),sqrt(v*v*dt*dt) );

			
			}
			robot::theta[0]=robot::theta[1];
		}
		
	}
	//Sin multiplication
	else if(mode> CR_P)
	{
		double A = 0.8*distribution(generator)+4.0;
		double f = 2.0*distribution(generator)+10.0;
		double k = 0.8*distribution(generator)+4.0;
		double d = 0.4*M_PI*distribution(generator);
		/*
		std::cout<<"Sin mult mode"<<std::endl;
		std::cout<<"Parameters:"<<std::endl;
		std::cout<<"A = "<<A<<std::endl;
		std::cout<<"d = "<<d<<std::endl;
		std::cout<<"f = "<<f<<std::endl;
		std::cout<<"k = "<<k<<std::endl;
		*/
		for(i=0;i<N;i++)
		{
		
			if(i==0)
			{
				robot::r[i] = l;
				
			}
			else
			{
				robot::r[i] = A*sin(f*i*dt)*sin(k*i*dt+d) + l;
				robot::theta[i] = atan2( (r[i]-r[i-1]),sqrt(v*v*dt*dt) );
				if(robot::r[i]<0)
					robot::r[i] = 0;
				else if(robot::r[i]>20)
					robot::r[i]=20;
			}
		}	robot::theta[0]=robot::theta[1];
	}
	
	//Offset with tanh convergance
	else if(mode<0)
	{
		double A = 0.5*distribution(generator);
		double B = 3*distribution(generator);
		double C = 0.1*distribution(generator);
		double f = 2.0*distribution(generator)+10.0;
		double k = 0.3*distribution(generator)+0.3;
		double d = 0.4*M_PI*distribution(generator);
		
		/*
		std::cout<<"Tanh mode"<<std::endl;
		std::cout<<"Parameters:"<<std::endl;
		std::cout<<"A = "<<A<<std::endl;
		std::cout<<"B = "<<B<<std::endl;
		std::cout<<"C = "<<C<<std::endl;
		std::cout<<"d = "<<d<<std::endl;
		std::cout<<"f = "<<f<<std::endl;
		std::cout<<"k = "<<k<<std::endl;
			*/
		for(i=0;i<N;i++)
		{
		
			
				robot::r[i] = B*(tanh(k*i*dt)-1)+A*sin(f*i*dt+d)+C*pow(sin(f*i*dt+d),3) + l;
				if(robot::r[i]<0)
					robot::r[i] = 0;
				else if(robot::r[i]>20)
					robot::r[i]=20;
				if(i!=0)
					robot::theta[i] = atan2( (robot::r[i]-robot::r[i-1]),sqrt(v*v*dt*dt) );
	
		}
		robot::theta[0] = robot::theta[1];
	}
	//Sin Summation with 5^th order harmonics
	else
	{
		double A = 0.8*distribution(generator);
		double B = 1.5*distribution(generator);
		double C = 0.2*distribution(generator);
		double D = 0.1*distribution(generator);
		double f = 2.0*distribution(generator)+10.0;
		double k = 0.8*distribution(generator)+4;
		double d = 0.4*M_PI*distribution(generator);
		
		/*
		std::cout<<"Sin sum mode"<<std::endl;
		std::cout<<"Parameters:"<<std::endl;
		std::cout<<"A = "<<A<<std::endl;
		std::cout<<"B = "<<B<<std::endl;
		std::cout<<"C = "<<C<<std::endl;
		std::cout<<"D = "<<D<<std::endl;
		std::cout<<"d = "<<d<<std::endl;
		std::cout<<"f = "<<f<<std::endl;
		std::cout<<"k = "<<k<<std::endl;
		*/
		for(i=0;i<N;i++)
		{
		
				robot::r[i] = B*sin(k*i*dt+d)+A*sin(f*i*dt)+C*pow(sin(k*i*dt+4*d),3) +D*pow(sin(k*i*dt+4*d),5)+ l;
				if(robot::r[i]<0)
					robot::r[i] = 0;
				else if(robot::r[i]>20)
					robot::r[i]=20;
					
				if(i!=0)
					robot::theta[i] = atan2( (r[i]-r[i-1]),sqrt(v*v*dt*dt) );
	
			
		}
		robot::theta[0] = robot::theta[1];
	}
	
	std::cout<<"Path generated."<<std::endl;
	

}
void robot::print_path()
{
	int i;
	for(i=0;i<N;i++)
	{
		//std::cout<<"r : "<<robot::r[i]<<"     theta: "<<robot::theta[i]<<std::endl;
	}
		
	std::ofstream ofile;
	ofile.open("data.txt",std::ios::out);
	
	std::ostringstream strs;
	std::string str;
	
	for(i=0;i<N;i++)
	{
			strs << robot::r[i];
			str = strs.str();
			ofile<<str<<"   ";
			
			strs.str("");
			strs.clear();
			
			strs << robot::theta[i];
			str = strs.str();
			ofile<<str<<"   ";
			
			strs.str("");
			strs.clear();

					
		ofile<<std::endl;
		
	}

	ofile.close();
}

double *robot::read_sens1()
{
	
	unsigned seed = std::chrono::system_clock::now().time_since_epoch().count();
  	std::default_random_engine generator (seed);
  	std::normal_distribution<double> distribution(0.0,sigma);
	
	int i;
	double *data = (double *)malloc(1000*sizeof(double));
	double k,nois;
	nois = distribution(generator);
	
	if(robot::sens1_pos == -999)
	{
		for(i=0;i<N;i++)
			data[i] = -999;
	}
	
	for(i=0;i<N;i++)
	{
		k = sin(sens1_pos+robot::theta[i]);
	
		if(k==0)
		{
			data[i] = MAX_R;
		}
		else
		{
		
			if(k>0)
			{
				data[i] = robot::r[i]/k + nois;
				//std::cout<<"In positive, data = "<<data[i]<<" k = "<<k<<" nois = "<<nois;
			}
			else
			{
				data[i] = (L-robot::r[i])/(-k) + nois;
				//std::cout<<"In negative, data = "<<data[i]<<" k = "<<k<<" nois = "<<nois;
			}
						
		}
		
		double c = cos(sens1_pos+robot::theta[i]);
		
		if( c>0)
		{	
		
			double wall_d = (robot::wall_pos-i)*v*dt;
			if(data[i]>(wall_d/c))
			{
				
				data[i] = wall_d/c+nois;
				//std::cout<<"and changing data to "<<data[i];
			}
		}
		
		if(data[i]>MAX_R)
			data[i] = MAX_R;
		
		
		if(i>=robot::wall_pos)
		{
			data[i] = -1;
		}
		//std::cout<<std::endl;
	}
	
	return data;
}


double *robot::read_sens2()
{
	unsigned seed = std::chrono::system_clock::now().time_since_epoch().count();
  	std::default_random_engine generator (seed);
  	std::normal_distribution<double> distribution(0.0,sigma);

	int i,nois;
	double *data = (double *)malloc(1000*sizeof(double));
	double k;
	
		
	if(robot::sens1_pos == -999)
	{
		for(i=0;i<N;i++)
			data[i] = -999;
	}
	
	for(i=0;i<N;i++)
	{
		k = sin(sens2_pos+robot::theta[i]);
	
		if(k==0)
		{
			data[i] = MAX_R;
		}
		else
		{
			nois = distribution(generator);
			if(k>0)
			{
				data[i] = robot::r[i]/k + nois;
			}
			else
			{
				data[i] = (L-robot::r[i])/(-k) + nois;
			}
						

		}
		
		double c = cos(sens2_pos+robot::theta[i]);
		
		if( c>0)
		{
			double wall_d = (robot::wall_pos-i)*v*dt;
			if(data[i]>(wall_d/c))
			{
				data[i] = wall_d/c;
			}
		}
		
		if(data[i]>MAX_R)
			data[i] = MAX_R;
			
		if(i>=robot::wall_pos)
		{
			data[i] = -1;
		}
	}
	return data;
}

double *robot::read_sens3()
{
	unsigned seed = std::chrono::system_clock::now().time_since_epoch().count();
  	std::default_random_engine generator (seed);
  	std::normal_distribution<double> distribution(0.0,sigma);

	
	int i,nois;
	double *data = (double *)malloc(1000*sizeof(double));
	double k;
	
		
	if(robot::sens1_pos == -999)
	{
		for(i=0;i<N;i++)
			data[i] = -999;
	}
	
	for(i=0;i<N;i++)
	{
		k = sin(sens3_pos+robot::theta[i]);
		
		if(k==0)
		{
			data[i] = MAX_R;
		}
		else
		{
			nois = distribution(generator);
			if(k>0)
			{
				data[i] = robot::r[i]/k + nois;
			}
			else
			{
				data[i] = (L-robot::r[i])/(-k) + nois;
			}
						
		}
		double c = cos(sens3_pos+robot::theta[i]);
		
		if( c>0)
		{
			double wall_d = (robot::wall_pos-i)*v*dt;
			if(data[i]>(wall_d/c))
			{
				data[i] = wall_d/c;
			}
		}
		
		if(data[i]>MAX_R)
			data[i] = MAX_R;
			
		
		if(i>=robot::wall_pos)
		{
			data[i] = -1;
		}		
	}
	return data;
}


double *robot::read_sens4()
{
	
	unsigned seed = std::chrono::system_clock::now().time_since_epoch().count();
  	std::default_random_engine generator (seed);
  	std::normal_distribution<double> distribution(0.0,sigma);

	double nois;
	
	int i;
	double *data = (double *)malloc(1000*sizeof(double));
	double k;
	
		
	if(robot::sens1_pos == -999)
	{
		for(i=0;i<N;i++)
			data[i] = -999;
	}
	
	for(i=0;i<N;i++)
	{
		k = sin(sens4_pos+robot::theta[i]);
		if(k==0)
		{
			data[i] = MAX_R;
		}
		else
		{
			nois = distribution(generator); 
			if(k>0)
			{
				data[i] = robot::r[i]/k + nois;
			}
			else
			{
				data[i] = (L-robot::r[i])/(-k) + nois;
			}

		}
		
		double c = cos(sens4_pos+robot::theta[i]);
		
		if( c>0)
		{
			double wall_d = (robot::wall_pos-i)*v*dt;
			if(data[i]>(wall_d/c))
			{
				data[i] = wall_d/c;
			}
		}
		
		if(data[i]>MAX_R)
			data[i] = MAX_R;
			
		if(i>=robot::wall_pos)
		{
			data[i] = -1;
		}

	}
	return data;
}

void robot::print_prediction(double *pred_r, double *pred_theta)
{
	int i;
	

	std::ofstream ofile;
	ofile.open("prediction.txt",std::ios::out);
	
	std::ostringstream strs;
	std::string str;
	
	for(i=0;i<N;i++)
	{
			strs << pred_r[i];
			str = strs.str();
			ofile<<str<<"   ";
			
			strs.str("");
			strs.clear();
			
			strs << pred_theta[i];
			str = strs.str();
			ofile<<str<<"   ";
			
			strs.str("");
			strs.clear();

					
		ofile<<std::endl;
		
	}

	ofile.close();
	
}


double robot::MSE_r(double *pred_r)
{
	int i=0;
	double er = 0;
	int n = N;
	if(robot::wall_pos <N)
		n = robot::wall_pos;
	for(i=0;i<n;i++)
	{ 
		er += pow((robot::r[i]-pred_r[i]),2);
		
	}
	er = er/n;
	return er;
}
double robot::MSE_theta(double *pred_theta)
{
	int i=0;
	double er = 0;
	
	int n = N;
	if(robot::wall_pos <N)
		n = robot::wall_pos;
	
	for(i=0;i<n;i++)
	{ 
		er += pow((robot::theta[i]-pred_theta[i]),2);	
	}
	er = er/n;
	return er;
}

void robot::print_wall_pos()
{
	double wall_d = robot::wall_pos*dt*v;
	std::cout<<"Position of the wall was at x = "<<wall_d<<" cm at t index = "<<robot::wall_pos<<std::endl;
}

void robot::stop(int t_s)
{
	if(t_s>robot::wall_pos)
		std::cout<<"The robot has stopped "<<(t_s-robot::wall_pos)*dt*v<<" cm too late!"<<std::endl;
	
	else
		std::cout<<"The robot has stopped "<<(robot::wall_pos-t_s)*dt*v<<" cm before the wall."<<std::endl;
}


}
