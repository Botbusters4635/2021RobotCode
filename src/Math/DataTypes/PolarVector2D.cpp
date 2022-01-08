//
// Created by Abiel on 8/9/18.
//

#include "Math/DataTypes/PolarVector2D.h"

PolarVector2D::PolarVector2D(double magnitude, double angle, const Point2D &originPoint) {
	this->magnitude = magnitude;
	this->angle = angle;
	
	this->originPoint = originPoint;
}

PolarVector2D::PolarVector2D(double magnitude, double angle) : PolarVector2D(magnitude, angle, Point2D(0.0, 0.0)) {
	;
}

PolarVector2D::PolarVector2D() : PolarVector2D(0.0, 0.0, Point2D(0.0, 0.0)) {
	;
}

PolarVector2D::PolarVector2D(const Point2D &originPoint, const Point2D &endPoint) : PolarVector2D(
		originPoint.getVector(endPoint)) {
	;
}

PolarVector2D::PolarVector2D(const Point2D &endPoint) : PolarVector2D(endPoint.getVector()) {
	;
}

void PolarVector2D::setMagnitude(double magnitude) {
	this->magnitude = magnitude;
}

void PolarVector2D::setAngle(double angle) {
	this->angle = angle;
}

double PolarVector2D::getMagnitude() const {
	return magnitude;
}

double PolarVector2D::getAngle() const {
	return angle;
}

Point2D PolarVector2D::getOriginPoint() const {
	return originPoint;
}

Point2D PolarVector2D::convertToPoint() const {
	return (Point2D(cos(angle) * magnitude, sin(angle) * magnitude) + originPoint);
}

PolarVector2D &PolarVector2D::operator+=(const PolarVector2D &rhs) {
	if (this->originPoint != rhs.getOriginPoint())
		throw std::logic_error("Tried to add vectors with different origins");
	
	Point2D newPoint = this->convertToPoint() + rhs.convertToPoint();
	
	*this = PolarVector2D(this->originPoint, newPoint);
	
	return *this;
}

const PolarVector2D PolarVector2D::operator+(const PolarVector2D &rhs) const {
	return PolarVector2D(*this) += rhs;
}

PolarVector2D &PolarVector2D::operator-=(const PolarVector2D &rhs) {
	if (this->originPoint != rhs.getOriginPoint())
		throw std::logic_error("Tried to subtract vectors with different origins");
	
	Point2D newPoint = this->convertToPoint() - rhs.convertToPoint();
	
	*this = PolarVector2D(this->originPoint, newPoint);
	
	return *this;
}

const PolarVector2D PolarVector2D::operator-(const PolarVector2D &rhs) const {
	return PolarVector2D(*this) -= rhs;
}

PolarVector2D &PolarVector2D::operator*=(double rhs) {
	this->magnitude *= rhs;
	
	return *this;
}

const PolarVector2D PolarVector2D::operator*(double rhs) const {
	return PolarVector2D(*this) *= rhs;
}

PolarVector2D &PolarVector2D::operator/=(double rhs) {
	this->magnitude /= rhs;
	
	return *this;
}

const PolarVector2D PolarVector2D::operator/(double rhs) const {
	return PolarVector2D(*this) /= rhs;
}

bool PolarVector2D::operator==(const PolarVector2D &other) const {
	return (
			this->magnitude == other.getMagnitude() &&
			this->angle == other.getAngle() &&
			this->originPoint == other.getOriginPoint());
}

bool PolarVector2D::operator!=(const PolarVector2D &other) const {
	return (!(*this == other));
}