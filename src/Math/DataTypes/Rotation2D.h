//
// Created by Abiel on 8/20/18.
//

#ifndef BOTBUSTERSREBIRTH_ROTATION2D_H
#define BOTBUSTERSREBIRTH_ROTATION2D_H

#include "Point2D.h"
#include <cmath>
#include <limits>

class Rotation2D {
public:
	Rotation2D(double x, double y, bool norm);
	
	Rotation2D(const Point2D &direction, bool norm);
	
	Rotation2D(const Rotation2D &other);
	
	explicit Rotation2D(double radians);
	
	Rotation2D();
	
	static Rotation2D fromRadians(double radians);
	
	double getCos() const;
	
	double getSin() const;
	
	double getTan() const;
	
	double getRadians() const;
	
	Rotation2D rotateBy(const Rotation2D &other) const;
	
	Rotation2D normal() const;
	
	Rotation2D inverse() const;
	
	bool isParallel(const Rotation2D &other) const;
	
	Point2D toPoint() const;
	
	//Interpolates a value between the two rotations (uses x from 0 to 1)
	Rotation2D interpolate(const Rotation2D &other, double x);
	
	//Gets the distance between two rotations
	double distance(const Rotation2D &other) const;
	
	bool operator==(const Rotation2D &other) const;
	
	bool operator!=(const Rotation2D &other) const;
	
	Rotation2D &operator=(const Rotation2D &other);
	
	friend std::ostream &operator<<(std::ostream &os, Rotation2D const &rotation);
	
	friend std::istream &operator>>(std::istream &in, Rotation2D &rotation);

private:
	double cos_angle;
	double sin_angle;
};


#endif //BOTBUSTERSREBIRTH_ROTATION2D_H
