//============================================================================
// Name        : OctoROACH_Framework.cpp
// Author      : Luis Valbuena
// Version     :
// Copyright   : Marhes laboratory UNM
// Description : OctoROACH Framework.
//============================================================================

#include "vicon_pos.h"
#include "stdlib.h"
#include "Client.h"
#include "librerialuis.h"
#include "math.h"

#include <boost/python.hpp>
using namespace boost::python;

using namespace std;

//#define ROBOTIP  "localhost"
//#define ROBOTPORT 6665


struct cpluspluscode_Wrap4Python
{
	cpluspluscode_Wrap4Python()
	{
		a_11= -0.019709753875392;
		a_21= -0.015173074449847;
		a_31= -0.999690603843965;
		a_12= -0.861839978997550;
		a_22= -0.506579426897183;
		a_32=  0.024680657325591;
		a_13= -0.506797174620847;
		a_23=  0.862059778702301;
		a_33= -0.003092206342704;

		std::string Models[4];
		std::string temp;
		
		temp="OctoROACH";
		Models[0]=temp;

		temp="OctoROACH2";
		Models[1]=temp;

		temp="quadrotor_2b";
		Models[2]=temp;

		temp="quadrotor_3";
		Models[3]=temp;

		//v = new vicon_pos("OctoROACH","OctoROACH");
		v = new vicon_pos();
		v->update();
		indexx_OCTO=v->find_sub("OctoROACH");
		indexx_OCTO2=v->find_sub("OctoROACH2");
	 	indexx_Quad1=v->find_sub("quadrotor_2b");
	 	indexx_Quad2=v->find_sub("quadrotor_3");
		cout << "Index OctoROACH = " << indexx_OCTO <<", "<< "Index OctoROACH2 = "<< indexx_OCTO2 <<", "<< "Index quad 1 = "<< indexx_Quad1 <<", "<< "Index quad 2 = "<< indexx_Quad2 <<", "  <<"\n";
	}
	void retrievePositioning()
	{

		float s_euler_x,c_euler_x,s_euler_y,c_euler_y,s_euler_z,c_euler_z;
		float r_11,r_12,r_13,r_21,r_22,r_23,r_31,r_32,r_33;
		float euler_x,euler_y,euler_z;
		float tempx,tempy;

		//cout << "index = " << indexx << " \n";

		v->update();

///////////////////////OCTOROACH_ONE////////////////////
		v->get_subject_marker(indexx_OCTO,"Origin",tempx,tempy,pos_z_octo1);
		v->get_subject_euler_xyz(indexx_OCTO,euler_x,euler_y,euler_z);

		s_euler_x=sin(euler_x);
		c_euler_x=cos(euler_x);
		s_euler_y=sin(euler_y);
		c_euler_y=cos(euler_y);
		s_euler_z=sin(euler_z);
		c_euler_z=cos(euler_z);

		r_11=c_euler_y*c_euler_z;
		r_12=-c_euler_y*s_euler_z;
		r_13=s_euler_y;
		r_21=c_euler_x*s_euler_z + c_euler_z*s_euler_x*s_euler_y;
		r_22=c_euler_x*c_euler_z - s_euler_x*s_euler_y*s_euler_z;
		r_23=-c_euler_y*s_euler_x;
		r_31=s_euler_x*s_euler_z - c_euler_x*c_euler_z*s_euler_y;
		r_32=c_euler_z*s_euler_x + c_euler_x*s_euler_y*s_euler_z;
		r_33=c_euler_x*c_euler_y;

		alpha_octo1=-atan2Mia(a_13*r_21 + a_23*r_22 + a_33*r_23,a_13*r_31 + a_23*r_32 + a_33*r_33);
		betha_octo1= asin( a_13*r_11 + a_23*r_12 + a_33*r_13 );
		theta_octo1=-atan2Mia(a_12*r_11 + a_22*r_12 + a_32*r_13,a_11*r_11 + a_21*r_12 + a_31*r_13);

		//Fixing origin offset
		pos_x_octo1 = (76.2*cos(theta_octo1)) + (20.3*sin(theta_octo1)) + tempx;
		pos_y_octo1 = (76.2*sin(theta_octo1)) + (-20.3*cos(theta_octo1)) + tempy;

	//cout << "position Octo 1= [" << pos_x_octo1<<" "<<pos_y_octo1<<" "<<pos_z_octo1 <<"]"<< "   orientation = ["<<(180/PI)*alpha_octo1<<" "<< (180/PI)*betha_octo1<<" "<< (180/PI)*theta_octo1 <<"]"  <<"\n";
///////////////////////OCTOROACH_ONE////////////////////


///////////////////////OCTOROACH_TWO////////////////////
		v->get_subject_marker(indexx_OCTO2,"Origin",tempx,tempy,pos_z_octo2);
		v->get_subject_euler_xyz(indexx_OCTO2,euler_x,euler_y,euler_z);

		s_euler_x=sin(euler_x);
		c_euler_x=cos(euler_x);
		s_euler_y=sin(euler_y);
		c_euler_y=cos(euler_y);
		s_euler_z=sin(euler_z);
		c_euler_z=cos(euler_z);

		r_11=c_euler_y*c_euler_z;
		r_12=-c_euler_y*s_euler_z;
		r_13=s_euler_y;
		r_21=c_euler_x*s_euler_z + c_euler_z*s_euler_x*s_euler_y;
		r_22=c_euler_x*c_euler_z - s_euler_x*s_euler_y*s_euler_z;
		r_23=-c_euler_y*s_euler_x;
		r_31=s_euler_x*s_euler_z - c_euler_x*c_euler_z*s_euler_y;
		r_32=c_euler_z*s_euler_x + c_euler_x*s_euler_y*s_euler_z;
		r_33=c_euler_x*c_euler_y;

		alpha_octo2=-atan2Mia(a_13*r_21 + a_23*r_22 + a_33*r_23,a_13*r_31 + a_23*r_32 + a_33*r_33);
		betha_octo2= asin( a_13*r_11 + a_23*r_12 + a_33*r_13 );
		theta_octo2=-atan2Mia(a_12*r_11 + a_22*r_12 + a_32*r_13,a_11*r_11 + a_21*r_12 + a_31*r_13);

		//Fixing origin offset
		pos_x_octo2 = (76.2*cos(theta_octo2)) + (20.3*sin(theta_octo2)) + tempx;
		pos_y_octo2 = (76.2*sin(theta_octo2)) + (-20.3*cos(theta_octo2)) + tempy;

	//cout << "position Octo 2= [" << pos_x_octo2<<" "<<pos_y_octo2<<" "<<pos_z_octo2 <<"]"<< "   orientation = ["<<(180/PI)*alpha_octo2<<" "<< (180/PI)*betha_octo2<<" "<< (180/PI)*theta_octo2 <<"]"  <<"\n";
///////////////////////OCTOROACH_TWO////////////////////


///////////////////////QUAD_ONE////////////////////
		v->get_subject_marker(indexx_Quad1,"CoG",pos_x_quad1,pos_y_quad1,pos_z_quad1);
		v->get_subject_euler_xyz(indexx_Quad1,alpha_quad1,betha_quad1,theta_quad1);

	//cout << "position Quad 1= [" << pos_x_quad1<<" "<<pos_y_quad1<<" "<<pos_z_quad1<<"]"<< "   orientation = ["<<(180/PI)*alpha_quad1<<" "<< (180/PI)*betha_quad1<<" "<< (180/PI)*theta_quad1<<"]"  <<"\n";
///////////////////////QUAD_ONE////////////////////

///////////////////////QUAD_TWO////////////////////
		v->get_subject_marker(indexx_Quad2,"CoG",pos_x_quad2,pos_y_quad2,pos_z_quad2);
		v->get_subject_euler_xyz(indexx_Quad2,alpha_quad2,betha_quad2,theta_quad2);

	//cout << "position Quad 2= [" << pos_x_quad2<<" "<<pos_y_quad2<<" "<<pos_z_quad2<<"]"<< "   orientation = ["<<(180/PI)*alpha_quad2<<" "<< (180/PI)*betha_quad2<<" "<< (180/PI)*theta_quad2<<"]"  <<"\n";
///////////////////////QUAD_TWO////////////////////
	}
	double octo1_return_pos_x(){return double(pos_x_octo1);}
	double octo1_return_pos_y(){return double(pos_y_octo1);}
	double octo1_return_pos_z(){return double(pos_z_octo1);}
	double octo1_return_orientation_alpha(){return double(alpha_octo1);}
	double octo1_return_orientation_betha(){return double(betha_octo1);}
	double octo1_return_orientation_theta(){return theta_octo1;}

	double octo2_return_pos_x(){return double(pos_x_octo2);}
	double octo2_return_pos_y(){return double(pos_y_octo2);}
	double octo2_return_pos_z(){return double(pos_z_octo2);}
	double octo2_return_orientation_alpha(){return double(alpha_octo2);}
	double octo2_return_orientation_betha(){return double(betha_octo2);}
	double octo2_return_orientation_theta(){return theta_octo2;}

	double quad1_return_pos_x(){return double(pos_x_quad1);}
	double quad1_return_pos_y(){return double(pos_y_quad1);}
	double quad1_return_pos_z(){return double(pos_z_quad1);}
	double quad1_return_orientation_alpha(){return double(alpha_quad1);}
	double quad1_return_orientation_betha(){return double(betha_quad1);}
	double quad1_return_orientation_theta(){return theta_quad1;}	

	double quad2_return_pos_x(){return double(pos_x_quad2);}
	double quad2_return_pos_y(){return double(pos_y_quad2);}
	double quad2_return_pos_z(){return double(pos_z_quad2);}
	double quad2_return_orientation_alpha(){return double(alpha_quad2);}
	double quad2_return_orientation_betha(){return double(betha_quad2);}
	double quad2_return_orientation_theta(){return theta_quad2;}	

	float a_11,a_12,a_13,a_21,a_22,a_23,a_31,a_32,a_33;
	float pos_x_octo1,pos_y_octo1,pos_z_octo1;
	float alpha_octo1,betha_octo1,theta_octo1;

	float pos_x_octo2,pos_y_octo2,pos_z_octo2;
	float alpha_octo2,betha_octo2,theta_octo2;

	float pos_x_quad1,pos_y_quad1,pos_z_quad1;
	float alpha_quad1,betha_quad1,theta_quad1;

	float pos_x_quad2,pos_y_quad2,pos_z_quad2;
	float alpha_quad2,betha_quad2,theta_quad2;

	int indexx_OCTO;
	int indexx_OCTO2;
	int indexx_Quad1;
	int indexx_Quad2;

	vicon_pos* v;
};

BOOST_PYTHON_MODULE(cpluspluscode_Wrap4Python_ext)
{
	class_<cpluspluscode_Wrap4Python>("cpluspluscode_Wrap4Python", init<>())
			.def("retrievePositioning", &cpluspluscode_Wrap4Python::retrievePositioning)
			.def("octo1_return_pos_x",&cpluspluscode_Wrap4Python::octo1_return_pos_x)
			.def("octo1_return_pos_y",&cpluspluscode_Wrap4Python::octo1_return_pos_y)
			.def("octo1_return_pos_z",&cpluspluscode_Wrap4Python::octo1_return_pos_z)
			.def("octo1_return_orientation_alpha",&cpluspluscode_Wrap4Python::octo1_return_orientation_alpha)
			.def("octo1_return_orientation_betha",&cpluspluscode_Wrap4Python::octo1_return_orientation_betha)
			.def("octo1_return_orientation_theta",&cpluspluscode_Wrap4Python::octo1_return_orientation_theta)
			.def("octo2_return_pos_x",&cpluspluscode_Wrap4Python::octo2_return_pos_x)
			.def("octo2_return_pos_y",&cpluspluscode_Wrap4Python::octo2_return_pos_y)
			.def("octo2_return_pos_z",&cpluspluscode_Wrap4Python::octo2_return_pos_z)
			.def("octo2_return_orientation_alpha",&cpluspluscode_Wrap4Python::octo2_return_orientation_alpha)
			.def("octo2_return_orientation_betha",&cpluspluscode_Wrap4Python::octo2_return_orientation_betha)
			.def("octo2_return_orientation_theta",&cpluspluscode_Wrap4Python::octo2_return_orientation_theta)
			.def("quad1_return_pos_x",&cpluspluscode_Wrap4Python::quad1_return_pos_x)
			.def("quad1_return_pos_y",&cpluspluscode_Wrap4Python::quad1_return_pos_y)
			.def("quad1_return_pos_z",&cpluspluscode_Wrap4Python::quad1_return_pos_z)
			.def("quad1_return_orientation_alpha",&cpluspluscode_Wrap4Python::quad1_return_orientation_alpha)
			.def("quad1_return_orientation_betha",&cpluspluscode_Wrap4Python::quad1_return_orientation_betha)
			.def("quad1_return_orientation_theta",&cpluspluscode_Wrap4Python::quad1_return_orientation_theta)
			.def("quad2_return_pos_x",&cpluspluscode_Wrap4Python::quad2_return_pos_x)
			.def("quad2_return_pos_y",&cpluspluscode_Wrap4Python::quad2_return_pos_y)
			.def("quad2_return_pos_z",&cpluspluscode_Wrap4Python::quad2_return_pos_z)
			.def("quad2_return_orientation_alpha",&cpluspluscode_Wrap4Python::quad2_return_orientation_alpha)
			.def("quad2_return_orientation_betha",&cpluspluscode_Wrap4Python::quad2_return_orientation_betha)
			.def("quad2_return_orientation_theta",&cpluspluscode_Wrap4Python::quad2_return_orientation_theta)
	;
}
/*
int main(){

	float s_euler_x,c_euler_x,s_euler_y,c_euler_y,s_euler_z,c_euler_z;
	float a_11,a_12,a_13,a_21,a_22,a_23,a_31,a_32,a_33;
	float r_11,r_12,r_13,r_21,r_22,r_23,r_31,r_32,r_33;

	float alpha,betha,theta;

	a_11= -0.019709753875392;
	a_21= -0.015173074449847;
	a_31= -0.999690603843965;
	a_12= -0.861839978997550;
	a_22= -0.506579426897183;
	a_32=  0.024680657325591;
	a_13= -0.506797174620847;
	a_23=  0.862059778702301;
	a_33= -0.003092206342704;


	int a=0, indexx;

	float pos_x,pos_y,pos_z,euler_x,euler_y,euler_z;

	vicon_pos* v = new vicon_pos("OctoROACH","OctoROACH");
	v->update();

	indexx=v->find_sub("OctoROACH");
	cout << "index = " << indexx << " \n";

	for(a=0;a<500;a++)
	{
		v->update();

		//v->get_subject_coord(indexx,pos_x,pos_y,pos_z);

		v->get_subject_marker(indexx,"Origin",pos_x,pos_y,pos_z);
		v->get_subject_euler_xyz(indexx,euler_x,euler_y,euler_z);

		s_euler_x=sin(euler_x);
		c_euler_x=cos(euler_x);
		s_euler_y=sin(euler_y);
		c_euler_y=cos(euler_y);
		s_euler_z=sin(euler_z);
		c_euler_z=cos(euler_z);


		r_11=c_euler_y*c_euler_z;
		r_12=-c_euler_y*s_euler_z;
		r_13=s_euler_y;
		r_21=c_euler_x*s_euler_z + c_euler_z*s_euler_x*s_euler_y;
		r_22=c_euler_x*c_euler_z - s_euler_x*s_euler_y*s_euler_z;
		r_23=-c_euler_y*s_euler_x;
		r_31=s_euler_x*s_euler_z - c_euler_x*c_euler_z*s_euler_y;
		r_32=c_euler_z*s_euler_x + c_euler_x*s_euler_y*s_euler_z;
		r_33=c_euler_x*c_euler_y;

		alpha=-atan2Mia(a_13*r_21 + a_23*r_22 + a_33*r_23,a_13*r_31 + a_23*r_32 + a_33*r_33);
		betha= asin( a_13*r_11 + a_23*r_12 + a_33*r_13 );
		theta=-atan2Mia(a_12*r_11 + a_22*r_12 + a_32*r_13,a_11*r_11 + a_21*r_12 + a_31*r_13);

		cout << "position = [" << pos_x<<" "<<pos_y<<" "<<pos_z <<"]"<< "   orientation("<< 1+a<<",:) = ["<<(180/PI)*alpha<<" "<< (180/PI)*betha<<" "<< (180/PI)*theta <<"]"  <<"\n";
	//	cout <<"orientation("<< 1+a<<",:) = ["<< euler_x<<" "<<euler_y<<" "<<euler_z <<"]"  <<"\n";
	}

	return 0;
}
*/
