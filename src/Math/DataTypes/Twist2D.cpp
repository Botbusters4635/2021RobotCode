//
// Created by Abiel on 8/21/18.
//

#include "Math/EctoMath.h"
#include "Math/DataTypes/Twist2D.h"

Twist2D::Twist2D(double dx, double dy, double dtheta) {
	this->dx = dx;
	this->dy = dy;
	this->dtheta = dtheta;
}

Twist2D::Twist2D(const Twist2D &twist) : Twist2D(twist.getDx(), twist.getDy(), twist.getDtheta()) {
	;
}

Twist2D::Twist2D() : Twist2D(0.0, 0.0, 0.0) {
	;
}

Twist2D &Twist2D::operator=(const Twist2D &twist) {
	dx = twist.getDx();
	dy = twist.getDy();
	dtheta = twist.getDtheta();
	return *this;
}

void Twist2D::setDx(double dx) {
	this->dx = dx;
}

void Twist2D::setDy(double dy) {
	this->dy = dy;
}

void Twist2D::setDtheta(double dtheta) {
	this->dtheta = dtheta;
}

double Twist2D::getDx() const {
	return dx;
}

double Twist2D::getDy() const {
	return dy;
}

double Twist2D::getDtheta() const {
	return dtheta;
}

Twist2D Twist2D::scaled(double scale) const {
	return {dx * scale, dy * scale, dtheta * scale};
}

double Twist2D::norm() const {
	if (dy == 0.0)
		return std::abs(dx);
	
	return hypot(dx, dy);
}

double Twist2D::curvature() const {
	if (std::abs(dtheta) < 0 && norm() < 0)
		return 0.0;
	
	return dtheta / norm();
}

void Twist2D::roundToEpsilon() {
	if (std::abs(dx) < epsilon) {
		dx = 0.0;
	}
	
	if (std::abs(dy) < epsilon) {
		dy = 0.0;
	}
	
	if (std::abs(dtheta) < epsilon) {
		dtheta = 0.0;
	}
}

Twist2D Twist2D::roundToEpsilon_copy(const Twist2D &twist) {
	Twist2D output(twist);
	output.roundToEpsilon();
	return output;
}

std::ostream &operator<<(std::ostream &os, const Twist2D &twist) {
	os << "dX: " << twist.getDx() << " dY: " << twist.getDy() << " dTheta: " << twist.getDtheta();
	return os;
}

Twist2D &Twist2D::operator+=(const Twist2D &rhs) {
	this->dx += rhs.getDx();
	this->dy += rhs.getDy();
	
	this->dtheta = EctoMath::wrapAngle(this->dtheta + rhs.getDtheta());
	
	return *this;
}

const Twist2D Twist2D::operator+(const Twist2D &rhs) const {
	return Twist2D(*this) += rhs;
}

Twist2D &Twist2D::operator-=(const Twist2D &rhs) {
	this->dx -= rhs.getDx();
	this->dy -= rhs.getDy();
	
	this->dtheta = EctoMath::wrapAngle(this->dtheta - rhs.getDtheta());
	
	return *this;
}

const Twist2D Twist2D::operator-(const Twist2D &rhs) const {
	return Twist2D(*this) -= rhs;
}

Twist2D &Twist2D::operator*=(const Twist2D &rhs) {
	this->dx *= rhs.getDx();
	this->dy *= rhs.getDy();
	
	//this->dtheta = EctoMath::wrapAngle(this->dtheta * rhs.getDtheta());
	
	return *this;
}

const Twist2D Twist2D::operator*(const Twist2D &rhs) const {
	return Twist2D(*this) *= rhs;
}

Twist2D &Twist2D::operator/=(const Twist2D &rhs) {
	this->dx /= rhs.getDx();
	this->dy /= rhs.getDy();
	
	//this->dtheta = EctoMath::wrapAngle(this->dtheta / rhs.getDtheta());
	
	return *this;
}

const Twist2D Twist2D::operator/(const Twist2D &rhs) const {
	return Twist2D(*this) /= rhs;
}

Twist2D &Twist2D::operator+=(double rhs) {
	(*this) += Twist2D(rhs, rhs, rhs);
	
	return *this;
}

const Twist2D Twist2D::operator+(double rhs) const {
	return Twist2D(*this) += rhs;
}

Twist2D &Twist2D::operator-=(double rhs) {
	(*this) -= Twist2D(rhs, rhs, rhs);
	
	return *this;
}

const Twist2D Twist2D::operator-(double rhs) const {
	return Twist2D(*this) -= rhs;
}

Twist2D &Twist2D::operator*=(double rhs) {
	(*this) *= Twist2D(rhs, rhs, rhs);
	
	return *this;
}

const Twist2D Twist2D::operator*(double rhs) const {
	return Twist2D(*this) *= rhs;
}

Twist2D &Twist2D::operator/=(double rhs) {
	(*this) += Twist2D(rhs, rhs, rhs);
	
	return *this;
}

const Twist2D Twist2D::operator/(double rhs) const {
	return Twist2D(*this) /= rhs;
}

bool Twist2D::operator==(const Twist2D &other) const {
	return (this->getDx() == other.getDx() and this->getDy() == other.getDy()) and
	       this->getDtheta() == other.getDtheta();
}

bool Twist2D::operator!=(const Twist2D &other) const {
	return !(*this == other);
}