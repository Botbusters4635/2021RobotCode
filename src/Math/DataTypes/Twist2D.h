//
// Created by Abiel on 8/21/18.
//

#ifndef BOTBUSTERSREBIRTH_TWIST2D_H
#define BOTBUSTERSREBIRTH_TWIST2D_H

#include <cmath>
#include <ostream>

class Point2D;

/**
 * A movement along an arc with a constant curvature and velocity
 */
class Twist2D {
public:
	Twist2D(double dx, double dy, double dtheta);
	
	Twist2D(const Twist2D &twist);
	
	Twist2D();
	
	Twist2D &operator=(const Twist2D &twist);
	
	Twist2D scaled(double scale) const;
	
	double norm() const;
	
	double curvature() const;
	
	void setDx(double dx);
	
	void setDy(double dy);
	
	void setDtheta(double dtheta);
	
	double getDx() const;
	
	double getDy() const;
	
	double getDtheta() const;
	
	friend std::ostream &operator<<(std::ostream &os, const Twist2D &twist);
	
	void roundToEpsilon();
	
	static Twist2D roundToEpsilon_copy(const Twist2D &twist);
	
	Twist2D &operator+=(const Twist2D &rhs);
	
	const Twist2D operator+(const Twist2D &rhs) const;
	
	Twist2D &operator-=(const Twist2D &rhs);
	
	const Twist2D operator-(const Twist2D &rhs) const;
	
	Twist2D &operator*=(const Twist2D &rhs);
	
	const Twist2D operator*(const Twist2D &rhs) const;
	
	Twist2D &operator/=(const Twist2D &rhs);
	
	const Twist2D operator/(const Twist2D &rhs) const;
	
	Twist2D &operator+=(double rhs);
	
	const Twist2D operator+(double rhs) const;
	
	Twist2D &operator-=(double rhs);
	
	const Twist2D operator-(double rhs) const;
	
	Twist2D &operator*=(double rhs);
	
	const Twist2D operator*(double rhs) const;
	
	Twist2D &operator/=(double rhs);
	
	const Twist2D operator/(double rhs) const;
	
	bool operator==(const Twist2D &other) const;
	
	bool operator!=(const Twist2D &other) const;

private:
	static constexpr double epsilon = 0.0001;
	
	double dx, dy;
	double dtheta;
};


#endif //BOTBUSTERSREBIRTH_TWIST2D_H
