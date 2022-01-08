//
// Created by Abiel on 8/9/18.
//

#include "Math/DataTypes/RobotPose2D.h"
#include "Math/DataTypes/Twist2D.h"
#include "Math/EctoMath.h"

RobotPose2D::RobotPose2D(const Point2D &point, const Rotation2D &heading) : Point2D(point) {
	this->heading = heading;
}

RobotPose2D::RobotPose2D(double x, double y, const Rotation2D &heading) : Point2D(x, y) {
	this->heading = heading;
}

RobotPose2D::RobotPose2D(const Point2D &point, double theta) : RobotPose2D(point, Rotation2D::fromRadians(theta)) {
	;
}

RobotPose2D::RobotPose2D(double x, double y, double theta) : RobotPose2D(x, y, Rotation2D::fromRadians(theta)) {
	;
}

RobotPose2D::RobotPose2D() : RobotPose2D(0, 0, Rotation2D(0, 0, false)) {
	;
}

void RobotPose2D::setPoint(const Point2D &point) {
	this->x = point.getX();
	this->y = point.getY();
}

double RobotPose2D::getTheta() const {
	return heading.getRadians();
}

void RobotPose2D::setTheta(double theta) {
	this->heading = Rotation2D::fromRadians(theta);
}

Point2D RobotPose2D::getPoint() const {
	return {x, y};
}

Rotation2D RobotPose2D::getHeading() const {
	return heading;
}

void RobotPose2D::setHeading(const Rotation2D &heading) {
	this->heading = heading;
}

bool RobotPose2D::isColinear(const RobotPose2D &other) const {
	//Checks if headings are parallel
	if (!(this->getHeading().isParallel(other.getHeading())))
		return false;
	
	const Twist2D twist = log(inverse().transformBy(other));
	return (std::abs(twist.getDy()) < 1e-6 and std::abs(twist.getDtheta()) < 1e-6);
}

RobotPose2D RobotPose2D::transformBy(const RobotPose2D &other) const {
	return {this->Point2D::operator+(other).rotateBy(heading), heading.rotateBy(other.getHeading())};
}

RobotPose2D RobotPose2D::exp(const Twist2D &delta) {
	double sin_theta = sin(delta.getDtheta());
	double cos_theta = cos(delta.getDtheta());
	
	double s, c;
	
	if (std::abs(delta.getDtheta()) < 1e-6) {
		s = 1.0 - 1.0 / 6.0 * std::pow(delta.getDtheta(), 2.0);
		c = 0.5 * delta.getDtheta();
	} else {
		s = sin_theta / delta.getDtheta();
		c = (1.0 - cos_theta) / delta.getDtheta();
	}
	
	return {
			Point2D(delta.getDx() * s - delta.getDy() * c,
			        delta.getDx() * c + delta.getDy() * s),
			Rotation2D(cos_theta, sin_theta, false)};
}

Twist2D RobotPose2D::log(const RobotPose2D &transform) {
	const double dtheta = transform.getHeading().getRadians();
	const double half_dtheta = dtheta / 2.0;
	
	double halftheta_by_tan_of_halfdtheta;
	
	if (std::abs(transform.getHeading().getCos() - 1.0) < 1e-6)
		halftheta_by_tan_of_halfdtheta = 1.0 - 1.0 / 12.0 * std::pow(dtheta, 2.0);
	else
		halftheta_by_tan_of_halfdtheta =
				-(half_dtheta * transform.getHeading().getSin()) / (transform.getHeading().getCos() - 1.0);
	
	Point2D translation_final = transform.getPoint().rotateBy(
			Rotation2D(halftheta_by_tan_of_halfdtheta, -half_dtheta, false));
	
	return {translation_final.getX(), translation_final.getY(), dtheta};
}

RobotPose2D RobotPose2D::inverse() const {
	Rotation2D rotation_inverted = heading.inverse();
	return {Point2D::inverse().rotateBy(rotation_inverted), rotation_inverted};
}

double RobotPose2D::getDistanceBetweenPoints(const RobotPose2D &firstPose, const RobotPose2D &secondPose) {
	return Point2D::getDistance(firstPose.getPoint(), secondPose.getPoint());
}

double RobotPose2D::getDistance(const RobotPose2D &other) const {
	return RobotPose2D::log(inverse().transformBy(other)).norm();
}

std::ostream &operator<<(std::ostream &os, const RobotPose2D &pose) {
	return os << pose.getPoint() << ',' << pose.getHeading();
}

std::istream &operator>>(std::istream &in, RobotPose2D &pose) {
	Point2D point;
	Rotation2D rotation;
	
	in >> point;
	in.ignore(); //,
	in >> rotation;
	
	pose = point;
	pose.setHeading(rotation);
	
	return in;
}

RobotPose2D &RobotPose2D::operator=(const Point2D &point) {
	this->x = point.getX();
	this->y = point.getY();
	return *this;
}

RobotPose2D &RobotPose2D::operator-=(const RobotPose2D &other) {
	this->x -= other.getX();
	this->y -= other.getY();
	
	double rotationResult = EctoMath::shortestAngleTurn(this->getHeading().getRadians(),
	                                                    other.getHeading().getRadians());
	this->setHeading(Rotation2D::fromRadians(rotationResult));
	
	return *this;
}

RobotPose2D RobotPose2D::operator-(const RobotPose2D &other) const {
	return RobotPose2D(*this) -= other;
}

RobotPose2D &RobotPose2D::operator+=(const RobotPose2D &other) {
	this->x += other.getX();
	this->y += other.getY();
	
	this->setHeading(Rotation2D::fromRadians(
			EctoMath::addTwoAngles(this->getHeading().getRadians(), other.getHeading().getRadians())));
	
	return *this;
}

RobotPose2D RobotPose2D::operator+(const RobotPose2D &other) const {
	return RobotPose2D(*this) += other;
}