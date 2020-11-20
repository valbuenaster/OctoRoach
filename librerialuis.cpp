/*
 * librerialuis.cpp
 *
 *  Created on: Jun 9, 2010
 *      Author: Luis Ariel Valbuena Reyes
 *
 */

//#include <iostream>
#include "stdio.h"
#include <stdlib.h>
#include <unistd.h>
#include <errno.h>
#include <string.h>
#include "math.h"

#include "librerialuis.h"

double atan2Mia(double *y,double *x)
{
	double temp;
	if((*x!=0)&&(*y!=0))
	{
		temp=acos(*x/sqrt(pow(*x,2)+pow(*y,2)))*((asin(*y/(sqrt(pow(*x,2)+pow(*y,2)))))/(sqrt(pow(atan(*y/(*x)),2))));
	}else if((*x==0) && (*y>0))
	{
		temp= PI/2;
	}else if((*x==0) && (*y<0))
	{
		temp= -PI/2;
	}else if((*y==0) && (*x<0))
	{
		temp= PI;
	}else if((*y==0) && (*x>0))
	{
		temp= 0;
	}
	return temp;
}
double atan2Mia(double y,double x)
{
	//std::cout << "x: " << x << " y: " << y << std::endl;
	double temp;
	if((x!=0)&&(y!=0))
	{
		temp= acos(x/sqrt(pow(x,2)+pow(y,2)))*((asin(y/(sqrt(pow(x,2)+pow(y,2)))))/(sqrt(pow(atan(y/x),2))));
	}else if((x==0) && (y>0))
	{
		temp= PI/2;
	}else if((x==0) && (y<0))
	{
		temp= -PI/2;
	}else if((y==0) && (x<0))
	{
		temp= PI;
	}else if((y==0) && (x>0))
	{
		temp= 0;
	}
	//std::cout << "temp: " << temp << std::endl;
	return temp;
}
double der_tetha_dx(double x,double y)
{
	double cuadradosXY,M,N,P,dM_dx,dN_dx,dP_dx;

	cuadradosXY=pow(x,2)+pow(y,2);
	M=acos(x/sqrt(cuadradosXY));
	N=asin(y/sqrt(cuadradosXY));
	P=pow(atan(y/x),2);

	dM_dx=(-1/sqrt(1-pow(x/sqrt(cuadradosXY),2)))*((sqrt(cuadradosXY)-((pow(x,2)/sqrt(cuadradosXY))))/cuadradosXY);
	dN_dx=(1/sqrt(1-pow(y/sqrt(cuadradosXY),2)))*((-(x)*(y)/sqrt(cuadradosXY))/cuadradosXY);
	dP_dx=2*atan(y/x)*(1/(1+pow(y/x,2)))*(-y/pow(x,2));

	return ((sqrt(P)*((M*dN_dx)+(N*dM_dx)))-(0.5*M*N*(1/sqrt(P))*dP_dx))/pow(sqrt(P),2);
}
double der_tetha_dy(double x,double y)
{
	double cuadradosXY,M,N,P,dM_dy,dN_dy,dP_dy;

	cuadradosXY=pow(x,2)+pow(y,2);
	M=acos(x/sqrt(cuadradosXY));
	N=asin(y/sqrt(cuadradosXY));
	P=pow(atan(y/x),2);

	dM_dy=(-1/sqrt(1-pow(x/sqrt(cuadradosXY),2)))*((-(x)*(y)/sqrt(cuadradosXY))/cuadradosXY);
	dN_dy=(1/sqrt(1-pow(y/sqrt(cuadradosXY),2)))*((sqrt(cuadradosXY)-((pow(y,2)/sqrt(cuadradosXY))))/cuadradosXY);
	dP_dy=2*atan(y/(x))*(1/(1+pow(y/(x),2)))*(1/(x));

	return ((sqrt(P)*((M*dN_dy)+(N*dM_dy)))-(0.5*M*N*(1/sqrt(P))*dP_dy))/pow(sqrt(P),2);
}


