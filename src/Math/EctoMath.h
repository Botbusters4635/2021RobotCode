//
// Created by Abiel on 10/30/17.
//

#ifndef BOTBUSTERSREBIRTH_ECTOMATH_H
#define BOTBUSTERSREBIRTH_ECTOMATH_H

#include <cmath>

using namespace std;

class Rotation2D;

namespace EctoMath {
	double radiansToDegrees(double radians);
	
	double degreesToRadians(double degrees);
	
	double addTwoAngles(double first, double second);
	
	double wrapAngle(double angle);
	
	double shortestAngleTurn(double current, double target);
	
	Rotation2D convertTo360(const Rotation2D &input);
	
	double sinc(double x);
	
	double cosc(double x);
}

#endif