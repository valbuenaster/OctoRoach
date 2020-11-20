/*
 * librerialuis.h
 *
 *  Created on: Jun 9, 2010
 *      Author: Luis Ariel Valbuena Reyes
 */

#ifndef LIBRERIALUIS_H_
#define LIBRERIALUIS_H_

const double PI= 3.141592653589793;

const double Kk = 9.8; //how steep the field is from the frontier
const double aAValue = .05; //small but not 0
const double Llambda = 1.0; //doesn't do much

double atan2Mia(double *,double *);
double atan2Mia(double ,double );
double der_tetha_dx(double ,double );
double der_tetha_dy(double ,double );

#endif /* LIBRERIALUIS_H_ */
