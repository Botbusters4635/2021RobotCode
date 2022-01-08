//
// Created by 4ndre on 04/10/2021.
//

#include <frc/Timer.h>
#include "PIDShooter.h"

PIDShooter::PIDShooter(const PIDShooterConfig &config) : System("PIDShooter") {
	
	this->config = config;
	this->motors = config.motors;
	
	table = ntInstance.GetTable("PIDShooter");
	
	frc::TrapezoidProfile<units::radians>::State goal;
	
	goal.position = units::radian_t(currentVelocityTarget);
	goal.velocity = units::radians_per_second_t(config.maxJerk);
	
	frc::TrapezoidProfile<units::radians>::State state;
	
	state.position = units::radian_t(0);
	state.velocity = units::radians_per_second_t(0);
	
	
	trapezoidProfile = std::make_unique<frc::TrapezoidProfile<units::radians >>(
			frc::TrapezoidProfile<units::radians>::Constraints(units::radians_per_second_t(config.maxAcceleration),
			                                                   units::radians_per_second_squared_t(config.maxJerk)),
			goal, state);
	
}

void PIDShooter::robotInit() {
	
	constraintsStartTime = frc::Timer::GetFPGATimestamp();
	
	motors[0]->setControlMode(MotorControlMode::Velocity);
	motors[0]->setPIDConfig(config.pidConfig, 0);
	motors[0]->setClosedLoopRampRate(0.1);
	motors[0]->setOpenLoopRampRate(0.01);
	motors[0]->setMotorCurrentLimit(20.0);
	motors[0]->enableCurrentLimit(true);
	
	int i = 0;
	for (auto &motor: motors) {
		if (i > 0) {
			motor->followMotor(*motors[0], config.isInverted[i]);
			log->info("Follow Motor configured");
		} else {
			motor->invertMotor(config.isInverted[i]);
			log->info("Master Motor configured");
		}
		i++;
	}
}

void PIDShooter::robotUpdate() {
	velocity = getVelocity();
	if (velocity != lastVelocity) {
		const double velocityDt = frc::Timer::GetFPGATimestamp() - lastVelocityTime;
		
		acceleration = (velocity - lastVelocity) / velocityDt;
		
		lastVelocity = velocity;
		lastAcceleration = acceleration;
		
		lastVelocityTime = frc::Timer::GetFPGATimestamp();
	}
	lastMotorVelocitySetPoint = trapezoidProfile->Calculate(
			units::second_t(
					frc::Timer::GetFPGATimestamp() - constraintsStartTime)).position.to<double>();
	
	table->GetEntry("Velocity").SetDouble(getVelocity());
	table->GetEntry("Acceleration").SetDouble(getAcceleration());
	table->GetEntry("Output").SetDouble(lastMotorVelocitySetPoint);
	
	motors[0]->set(lastMotorVelocitySetPoint);
}

void PIDShooter::disabledInit() {

}

void PIDShooter::disabledUpdate() {

}

void PIDShooter::setPIDConfig(const PIDConfig &pidConfig) {
	this->config.pidConfig = pidConfig;
	motors[0]->setPIDConfig(pidConfig, 0);
	
}

double PIDShooter::getVelocity() const {
	return (motors[0]->getVelocity() / config.gearReduction);
}

double PIDShooter::getAcceleration() const {
	return acceleration;
}

void PIDShooter::setSetpoint(double velocity) {
	velocity = -fabs(velocity);
	if (lastSetpoint == velocity) return;
	frc::TrapezoidProfile<units::radians>::State goal;
	
	goal.position = units::radian_t(velocity);
	goal.velocity = units::radians_per_second_t(0);
	
	frc::TrapezoidProfile<units::radians>::State state;
	
	state.position = units::radian_t(getVelocity());
	state.velocity = units::radians_per_second_t(0);
	
	trapezoidProfile = std::make_unique<frc::TrapezoidProfile<units::radians >>(
			frc::TrapezoidProfile<units::radians>::Constraints(units::radians_per_second_t(config.maxAcceleration),
			                                                   units::radians_per_second_squared_t(config.maxJerk)),
			goal, state);
	constraintsStartTime = frc::Timer::GetFPGATimestamp();
	lastSetpoint = velocity;
}
