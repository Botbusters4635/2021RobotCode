//
// Created by Abiel on 8/20/18.
//

#include "Math/DataTypes/Rotation2D.h"

Rotation2D::Rotation2D() : Rotation2D(1, 0, false) {
	;
}

Rotation2D::Rotation2D(double x, double y, bool norm) {
	if (norm) {
		double magnitude = hypot(x, y);
		
		if (magnitude > 0) {
			sin_angle = y / magnitude;
			cos_angle = x / magnitude;
		} else {
			sin_angle = 0;
			cos_angle = 0;
		}
	} else {
		cos_angle = x;
		sin_angle = y;
	}
}

Rotation2D::Rotation2D(const Point2D &direction, bool norm) : Rotation2D(direction.getX(), direction.getY(), norm) {
	;
}

Rotation2D::Rotation2D(double radians) : Rotation2D(cos(radians), sin(radians), false) {
	;
}

Rotation2D::Rotation2D(const Rotation2D &other) {
	this->cos_angle = other.getCos();
	this->sin_angle = other.getSin();
}

Rotation2D Rotation2D::fromRadians(double radians) {
	return Rotation2D(cos(radians), sin(radians), false);
}

double Rotation2D::getCos() const {
	return cos_angle;
}

double Rotation2D::getSin() const {
	return sin_angle;
}

double Rotation2D::getTan() const {
	if (cos_angle < 0) {
		if (sin_angle >= 0.0) {
			return std::numeric_limits<double>::infinity();
		} else {
			return std::numeric_limits<double>::infinity() * -1.0;
		}
	}
	
	return sin_angle / cos_angle;
}

double Rotation2D::getRadians() const {
	return atan2(sin_angle, cos_angle);
}

Rotation2D Rotation2D::rotateBy(const Rotation2D &other) const {
	return (
			Rotation2D(cos_angle * other.cos_angle - sin_angle * other.sin_angle,
			           cos_angle * other.sin_angle + sin_angle * other.cos_angle, true));
}

Rotation2D Rotation2D::normal() const {
	return Rotation2D(-sin_angle, cos_angle, false);
}

Rotation2D Rotation2D::inverse() const {
	return Rotation2D(cos_angle, -sin_angle, false);
}

bool Rotation2D::isParallel(const Rotation2D &other) const {
	return std::abs(Point2D::crossMultiply(this->toPoint(), other.toPoint())) <= 1e-6;
}

Point2D Rotation2D::toPoint() const {
	return {cos_angle, sin_angle};
}

Rotation2D Rotation2D::interpolate(const Rotation2D &other, double x) {
	if (x <= 0.0)
		return Rotation2D(*this);
	else if (x >= 1.0)
		return Rotation2D(other);
	
	double angle_diff = this->inverse().rotateBy(other).getRadians();
	return this->rotateBy(Rotation2D::fromRadians(angle_diff * x));
}

double Rotation2D::distance(const Rotation2D &other) const {
	return inverse().rotateBy(other).getRadians();
}

bool Rotation2D::operator==(const Rotation2D &other) const {
	return distance(other) < 1e-6;
}

bool Rotation2D::operator!=(const Rotation2D &other) const {
	return !(*this == other);
}

std::ostream &operator<<(std::ostream &os, const Rotation2D &rotation) {
	return os << rotation.getRadians();
}

std::istream &operator>>(std::istream &in, Rotation2D &rotation) {
	double radians;
	
	in >> radians;
	
	rotation = Rotation2D::fromRadians(radians);
	
	return in;
}

Rotation2D &Rotation2D::operator=(const Rotation2D &other) {
	this->cos_angle = other.getCos();
	this->sin_angle = other.getSin();
	return *this;
}
