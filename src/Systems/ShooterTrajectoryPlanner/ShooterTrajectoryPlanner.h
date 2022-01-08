////
//// Created by abiel on 1/16/20.
////
//
//#ifndef BOTBUSTERSREBIRTH_SHOOTERTRAJECTORYPLANNER_H
//#define BOTBUSTERSREBIRTH_SHOOTERTRAJECTORYPLANNER_H
//
//#include <optional>
//#include <cmath>
//#include <EctoMath/DataType/Point2D.h>
//
//struct ShooterTrajectoryPlannerConfig {
//	double firstAngle = M_PI;
//	double secondAngle  = M_PI_2;
//
//	double shooterHeight = 0.4;
//};
//
//struct ShooterTrajectoryPlannerOutput {
//	double linearVelocity = 0; //Linear velocity at which the ball needs to go
//	bool useSecondAngle = false; //Which angle to use
//};
//
//class ShooterTrajectoryPlanner {
//public:
//	ShooterTrajectoryPlanner(const ShooterTrajectoryPlannerConfig &config);
//
//	std::optional<ShooterTrajectoryPlannerOutput> calculateOutput(const Point2D &target, const ShooterTrajectoryPlannerOutput &lastOutput);
//	static std::optional<double> calculateSingleOutput(const Point2D &target, double angle, double shooterHeight);
//
//private:
//	ShooterTrajectoryPlannerConfig config;
//
//	const double gravity = 9.8;
//
//	const double angleChangeWeight = 0.25;
//	const double velocityChangeWeight = 0.25;
//};
//
//
//#endif //BOTBUSTERSREBIRTH_SHOOTERTRAJECTORYPLANNER_H
