//
// Created by Abiel on 8/9/18.
//

#ifndef BOTBUSTERSREBIRTH_ROBOTPOSE2D_H
#define BOTBUSTERSREBIRTH_ROBOTPOSE2D_H

#include "Point2D.h"
#include "Rotation2D.h"
#include <cmath>

class Twist2D;

/**
 * A point with an angle value
 */
class RobotPose2D : public Point2D {
public:
	RobotPose2D(const Point2D &point, const Rotation2D &heading);
	
	RobotPose2D(double x, double y, const Rotation2D &heading);
	
	RobotPose2D();
	
	//Depreciated
	RobotPose2D(const Point2D &point, double theta);
	
	RobotPose2D(double x, double y, double theta);
	
	Point2D getPoint() const;
	
	//DEPRECIATED (use get heading instead)
	double getTheta() const;
	
	void setPoint(const Point2D &point);
	
	//DEPRECIATED (use set heading instead)
	void setTheta(double theta);
	
	Rotation2D getHeading() const;
	
	void setHeading(const Rotation2D &heading);
	
	bool isColinear(const RobotPose2D &other) const;
	
	RobotPose2D transformBy(const RobotPose2D &other) const;
	
	/**
	 * Gets a RobotPose2D from a constant curve velocity
	 * @param delta
	 * @return
	 */
	static RobotPose2D exp(const Twist2D &delta);
	
	/**
	 * Gets a constant curve velocity from a RobotPose2D
	 * @param transform
	 * @return
	 */
	static Twist2D log(const RobotPose2D &transform);
	
	static double getDistanceBetweenPoints(const RobotPose2D &firstPose, const RobotPose2D &secondPose);
	
	double getDistance(const RobotPose2D &other) const;
	
	/**
	 * Gets the opposite
	 * @return
	 */
	RobotPose2D inverse() const;
	
	friend std::ostream &operator<<(std::ostream &os, RobotPose2D const &pose);
	
	friend std::istream &operator>>(std::istream &in, RobotPose2D &pose);
	
	RobotPose2D &operator=(const Point2D &point);
	
	RobotPose2D &operator-=(const RobotPose2D &other);
	
	RobotPose2D operator-(const RobotPose2D &other) const;
	
	RobotPose2D &operator+=(const RobotPose2D &other);
	
	RobotPose2D operator+(const RobotPose2D &other) const;

private:
	Rotation2D heading;
};

#endif //BOTBUSTERSREBIRTH_ROBOTPOSE2D_H
