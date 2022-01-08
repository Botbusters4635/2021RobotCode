//
// Created by Abiel on 8/9/18.
//

#include "Math/DataTypes/Point2D.h"
#include "Math/DataTypes/PolarVector2D.h"
#include "Math/DataTypes/Rotation2D.h"

Point2D::Point2D() {
	x = 0.0;
	y = 0.0;
}

Point2D::Point2D(double x, double y) {
	this->x = x;
	this->y = y;
}

Point2D::Point2D(const Point2D &start, const Point2D &end) : Point2D(end.getX() - start.getX(),
                                                                     end.getY() - start.getY()) {
	;
}

double Point2D::getDistance(const Point2D &firstPoint, const Point2D &secondPoint) {
	return hypot((firstPoint.getX() - secondPoint.getX()), (firstPoint.getY() - secondPoint.getY()));
}

double Point2D::getDistance(const Point2D &secondPoint) const {
	return (getDistance(*this, secondPoint));
}

double Point2D::getDistance() const {
	return getDistance(Point2D(0.0, 0.0));
}

double Point2D::getAngleFromPoints(const Point2D &initialPoint, const Point2D &targetPoint) {
	return atan2(targetPoint.getY() - initialPoint.getY(), targetPoint.getX() - initialPoint.getX());
}

double Point2D::getAngleFromPoints(const Point2D &targetPoint) const {
	return getAngleFromPoints(*this, targetPoint);
}

double Point2D::getAngleFromPoints() const {
	return getAngleFromPoints(Point2D(0.0, 0.0), *this);
}

PolarVector2D Point2D::getVector(const Point2D &secondPoint) const {
	Point2D angleCalculation = *this - secondPoint;
	
	double theta = atan2(angleCalculation.getY(), angleCalculation.getX());
	double distance = getDistance(secondPoint);
	
	return {distance, theta, *this};
}

PolarVector2D Point2D::getVector() const {
	return getVector(Point2D(0.0, 0.0));
}

void Point2D::setX(double x) {
	this->x = x;
}

void Point2D::setY(double y) {
	this->y = y;
}

double Point2D::getX() const {
	return x;
}

double Point2D::getY() const {
	return y;
}

double Point2D::crossMultiply(const Point2D &firstPoint, const Point2D &secondPoint) {
	return firstPoint.getX() * secondPoint.getY() - firstPoint.getY() * secondPoint.getX();
}

Point2D Point2D::rotateBy(const Rotation2D &rotation) const {
	return {this->x * rotation.getCos() - this->y * rotation.getSin(),
	        this->x * rotation.getSin() + this->y * rotation.getCos()};
}

Point2D Point2D::inverse() const {
	return {-x, -y};
}

Point2D &Point2D::operator+=(const Point2D &rhs) {
	this->x += rhs.getX();
	this->y += rhs.getY();
	
	return *this;
}

const Point2D Point2D::operator+(const Point2D &rhs) const {
	return Point2D(*this) += rhs;
}

Point2D &Point2D::operator-=(const Point2D &rhs) {
	this->x -= rhs.getX();
	this->y -= rhs.getY();
	
	return *this;
}

const Point2D Point2D::operator-(const Point2D &rhs) const {
	return Point2D(*this) -= rhs;
}

Point2D &Point2D::operator*=(const Point2D &rhs) {
	this->x *= rhs.getX();
	this->y *= rhs.getY();
	
	return *this;
}

Point2D &Point2D::operator*=(double rhs) {
	this->x *= rhs;
	this->y *= rhs;
	
	return *this;
}

const Point2D Point2D::operator*(const Point2D &rhs) const {
	return Point2D(*this) *= rhs;
}

const Point2D Point2D::operator*(double rhs) const {
	return Point2D(*this) *= rhs;
}

Point2D &Point2D::operator/=(const Point2D &rhs) {
	this->x /= rhs.getX();
	this->y /= rhs.getY();
	
	return *this;
}

Point2D &Point2D::operator/=(double rhs) {
	this->x /= rhs;
	this->y /= rhs;
	
	return *this;
}

const Point2D Point2D::operator/(const Point2D &rhs) const {
	return Point2D(*this) /= rhs;
}

const Point2D Point2D::operator/(double rhs) const {
	return Point2D(*this) /= rhs;
}

bool Point2D::operator==(const Point2D &other) const {
	return (this->x == other.getX()) && (this->y == other.getY());
}

bool Point2D::operator!=(const Point2D &other) const {
	return !(*this == other);
}

std::ostream &operator<<(std::ostream &os, const Point2D &point) {
	return os << point.getX() << ',' << point.getY();
}

std::istream &operator>>(std::istream &in, Point2D &point) {
	in >> point.x;
	in.ignore(); //,
	in >> point.y;
	return in;
}