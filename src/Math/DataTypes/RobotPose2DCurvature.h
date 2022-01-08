//
// Created by Abiel on 8/20/18.
//

#ifndef BOTBUSTERSREBIRTH_ROBOTPOSE2DCURVATURE_H
#define BOTBUSTERSREBIRTH_ROBOTPOSE2DCURVATURE_H

#include "Point2D.h"
#include "RobotPose2D.h"
#include "Rotation2D.h"

class RobotPose2DCurvature : public RobotPose2D {
public:
	RobotPose2DCurvature(const RobotPose2D &pose, double curvature, double dcurvature_ds);
	
	RobotPose2DCurvature(const RobotPose2D &pose, double curvature);
	
	RobotPose2DCurvature(const Point2D &point, const Rotation2D &rotation, double curvature, double dcurvature_ds);
	
	RobotPose2DCurvature(const Point2D &point, const Rotation2D &rotation, double curvature);
	
	RobotPose2DCurvature();
	
	void setCurvature(double curvature);
	
	void setDCurvature(double dcurvature);
	
	double getCurvature() const;
	
	double getDCurvature() const;

private:
	double curvature;
	double dcurvature_ds;
};

#endif //BOTBUSTERSREBIRTH_ROBOTPOSE2DCURVATURE_H
