////
//// Created by abiel on 1/16/20.
////
//
//#include "ShooterTrajectoryPlanner.h"
//
//ShooterTrajectoryPlanner::ShooterTrajectoryPlanner(const ShooterTrajectoryPlannerConfig &config) {
//	this->config = config;
//}
//
//std::optional<double> ShooterTrajectoryPlanner::calculateSingleOutput(const Point2D &target,
//                                                                                              double angle, double shooterHeight) {
//	double vTarget = -gravity * std::pow(target.getX(), 2.0) * (std::pow(std::tan(angle),2) + 1);
//	vTarget /= 2((target.getY() - shooterHeight) - target.getX() * std::tan(angle));
//	vTarget = std::sqrt(vTarget);
//
//	if(std::isnan(vTarget)){
//		return std::nullopt;
//	}
//
//	return vTarget;
//}
//
//std::optional<ShooterTrajectoryPlannerOutput> ShooterTrajectoryPlanner::calculateOutput(const Point2D &target,
//                                                                                        const ShooterTrajectoryPlannerOutput &lastOutput) {
//	auto firstAngleTarget = calculateSingleOutput(target, config.firstAngle, config.shooterHeight);
//	auto secondAngleTarget = calculateSingleOutput(target, config.secondAngle, config.shooterHeight);
//
//	if(firstAngleTarget == std::nullopt and secondAngleTarget == std::nullopt){
//		//No possible solution!
//		return {};
//	}
//
//	return {};
//}