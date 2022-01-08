//
// Created by Abiel on 8/9/18.
//

#ifndef BOTBUSTERSREBIRTH_POINT2D_H
#define BOTBUSTERSREBIRTH_POINT2D_H

#include <cmath>
#include <ostream>
#include <istream>

class PolarVector2D;

class Rotation2D;

/**
 * Point
 */
class Point2D {
public:
	Point2D();
	
	Point2D(double x, double y);
	
	Point2D(const Point2D &start, const Point2D &end);
	
	void setX(double x);
	
	void setY(double y);
	
	double getX() const;
	
	double getY() const;
	
	//Calculates the distance between two points
	static double getDistance(const Point2D &firstPoint, const Point2D &secondPoint);
	
	//Calculates the distance between the current point and the given point
	double getDistance(const Point2D &secondPoint) const;
	
	//Calculates the distance betweeen the current point and (0,0)
	double getDistance() const;
	
	//Calculates the angle between two points
	static double getAngleFromPoints(const Point2D &initialPoint, const Point2D &targetPoint);
	
	//Caluclates the angle between the current point and the second point
	double getAngleFromPoints(const Point2D &secondPoint) const;
	
	//Calculates the angle between (0,0) and the current point
	double getAngleFromPoints() const;
	
	//Returns a vector between two points (using this point as the origin)
	PolarVector2D getVector(const Point2D &secondPoint) const;
	
	//Returns a vector (using 0, 0 as the origin)
	PolarVector2D getVector() const;
	
	static double crossMultiply(const Point2D &firstPoint, const Point2D &secondPoint);
	
	Point2D rotateBy(const Rotation2D &rotation) const;
	
	Point2D inverse() const;
	
	Point2D &operator+=(const Point2D &rhs);
	
	const Point2D operator+(const Point2D &rhs) const;
	
	Point2D &operator-=(const Point2D &rhs);
	
	const Point2D operator-(const Point2D &rhs) const;
	
	Point2D &operator*=(const Point2D &rhs);
	
	Point2D &operator*=(double rhs);
	
	const Point2D operator*(const Point2D &rhs) const;
	
	const Point2D operator*(double rhs) const;
	
	Point2D &operator/=(const Point2D &rhs);
	
	Point2D &operator/=(double rhs);
	
	const Point2D operator/(const Point2D &rhs) const;
	
	const Point2D operator/(double rhs) const;
	
	bool operator==(const Point2D &other) const;
	
	bool operator!=(const Point2D &other) const;
	
	friend std::ostream &operator<<(std::ostream &os, Point2D const &point);
	
	friend std::istream &operator>>(std::istream &in, Point2D &point);

protected:
	double x, y;
};


#endif //BOTBUSTERSREBIRTH_POINT2D_H
