//
// Created by Abiel on 8/9/18.
//

#ifndef BOTBUSTERSREBIRTH_POLARVECTOR2D_H
#define BOTBUSTERSREBIRTH_POLARVECTOR2D_H

#include "Point2D.h"

/**
 * Polar vector (angle (in radians) and magnitude)
 */
class PolarVector2D {
public:
	PolarVector2D(double magnitude, double angle, const Point2D &originPoint);
	
	PolarVector2D(double magnitude, double angle);
	
	PolarVector2D();
	
	//Creates a vector from two points
	PolarVector2D(const Point2D &originPoint, const Point2D &endPoint);
	
	//Creates a vector given an end point, using (0,0) as the origin
	explicit PolarVector2D(const Point2D &endPoint);
	
	void setMagnitude(double magnitude);
	
	void setAngle(double angle);
	
	double getMagnitude() const;
	
	double getAngle() const;
	
	Point2D getOriginPoint() const;
	
	Point2D convertToPoint() const;
	
	PolarVector2D &operator+=(const PolarVector2D &rhs);
	
	const PolarVector2D operator+(const PolarVector2D &rhs) const;
	
	PolarVector2D &operator-=(const PolarVector2D &rhs);
	
	const PolarVector2D operator-(const PolarVector2D &rhs) const;
	
	PolarVector2D &operator*=(double rhs);
	
	const PolarVector2D operator*(double rhs) const;
	
	PolarVector2D &operator/=(double rhs);
	
	const PolarVector2D operator/(double rhs) const;
	
	bool operator==(const PolarVector2D &other) const;
	
	bool operator!=(const PolarVector2D &other) const;

private:
	double magnitude, angle;
	
	Point2D originPoint;
};


#endif //BOTBUSTERSREBIRTH_POLARVECTOR2D_H
