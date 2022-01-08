//
// Created by 4ndre on 04/10/2021.
//

#ifndef BOTBUSTERS_REBIRTH_PIDSHOOTER_H
#define BOTBUSTERS_REBIRTH_PIDSHOOTER_H


#include <Core/EctoModule/System.h>
#include <frc/controller/ProfiledPIDController.h>
#include <units/length.h>
#include <frc/trajectory/TrapezoidProfile.h>
#include <units/velocity.h>
#include <units/angular_acceleration.h>
#include <units/angular_velocity.h>
#include <Control/EctoPID/PIDConfig.h>
#include <Core/MotorHandler/EctoMotor/EctoMotor.h>
#include <Core/MotorHandler/MotorManager.h>
#include <networktables/NetworkTable.h>
#include <networktables/NetworkTableEntry.h>
#include <networktables/NetworkTableInstance.h>


struct PIDShooterConfig {
	PIDConfig pidConfig;
	
	double maxAcceleration = 40;
	double maxJerk = 30;
	
	std::vector<std::shared_ptr<EctoMotor>> motors;
	
	double kP = 0.00015;
	double kI = 0.000002;
	double kD = 0.0;
	
	double gearReduction = 1;
	
	std::vector<bool> isInverted;
};

class PIDShooter : public System {
public:
	
	explicit PIDShooter(const PIDShooterConfig &PIDconfig);
	
	void robotInit() override;
	
	void robotUpdate() override;
	
	void disabledInit() override;
	
	void disabledUpdate() override;
	
	void setPIDConfig(const PIDConfig &pidConfig);
	
	double getVelocity() const;
	
	double getAcceleration() const;
	
	void setSetpoint(double velocity);

private:
	
	PIDShooterConfig config;
	
	double velocity, acceleration;
	double lastVelocityTime, lastVelocity, lastAcceleration, lastSetpoint;
	double lastTime;
	
	double lastMotorVelocitySetPoint;
	nt::NetworkTableInstance ntInstance = nt::NetworkTableInstance::GetDefault();
	std::shared_ptr<nt::NetworkTable> table;
	
	double constraintsStartTime;
	
	std::vector<std::shared_ptr<EctoMotor>> motors;
	
	double currentVelocityTarget;
	
	std::unique_ptr<frc::TrapezoidProfile<units::radians>> trapezoidProfile;
};


#endif //BOTBUSTERS_REBIRTH_PIDSHOOTER_H
