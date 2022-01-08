#include "SwerveModule.h"

#include <utility>
#include "Math/EctoMath.h"

SwerveModule::SwerveModule(const std::string &name, const std::shared_ptr<EctoMotor> &steerMotor,
                           const std::shared_ptr<EctoMotor> &wheelMotor, const SwerveModuleConfig &config) {
	moduleName = name;
	
	steerP = config.steerPID.p;
	steerI = config.steerPID.i;
	steerD = config.steerPID.d;
	
	this->steerMotor = steerMotor;
	this->wheelMotor = wheelMotor;
	
	wheelMotor->setPIDConfig(config.wheelPID);
	
	steerMotor->enableBrakingOnIdle(true);
	wheelMotor->enableBrakingOnIdle(true);
	
	steerMotor->setMotorCurrentLimit(config.steerCurrentLimit);
	steerMotor->enableCurrentLimit(true);
	
	wheelMotor->setMotorCurrentLimit(config.wheelCurrentLimit);
	wheelMotor->enableCurrentLimit(true);
	
	steerMotor->setAnalogPositionConversionFactor(2 * M_PI / 3.33333333); //Potentiometer v/rot
	steerMotor->setAnalogSensorOffset(config.analogOffset);
	analogOffset = config.analogOffset;
	
	steerMotor->enableVoltageCompensation(12.0);
	wheelMotor->enableVoltageCompensation(12.0); //TODO remove me
	
	steerMotor->setFeedbackMode(MotorFeedbackMode::Potentiometer);
	steerMotor->setOpenLoopRampRate(0.115);
	
	wheelMotor->invertMotor(false);
	steerMotor->invertMotor(true);
	
	wheelMotor->setClosedLoopRampRate(0.05);
	wheelMotor->setOpenLoopRampRate(0.05);
	
	wheelCircumference = config.wheelCircumference;
	gearRatio = config.gearRatio;
	
	steerController = std::make_unique<frc2::PIDController>(
			steerP, steerI, steerD
	);
	
	steerController->EnableContinuousInput(-M_PI, M_PI);
	
	analogFilter = std::make_unique<frc::LinearFilter<double>>(
			frc::LinearFilter<double>::SinglePoleIIR(0.05,
			                                         units::millisecond_t(20))
	);
	
	pidNotifier = std::make_unique<frc::Notifier>([this, config] {
		double pos = this->steerMotor->getPosition();
		if (config.enableAnalogFilter) {
			pos = analogFilter->Calculate(pos);
		}
		
		double out = this->steerController->Calculate(pos);
		this->steerMotor->set(out, MotorControlMode::Percent);
	});
	
	table = nt::NetworkTableInstance::GetDefault().GetTable(fmt::format("SwerveModule/{}", moduleName));
	velocityEntry = table->GetEntry("Velocity");
	angleEntry = table->GetEntry("Angle");
	wheelSetpointEntry = table->GetEntry("WheelSetpoint");
	steerSetpointEntry = table->GetEntry("SteerSetpoint");
	controlModeEntry = table->GetEntry("ControlMode");
	encoderState = table->GetEntry("EncoderPosition");
	rawAnalog = table->GetEntry("RawAnalog");
	
	ntNotifier = std::make_unique<frc::Notifier>([this] {
		this->updateNT();
	});
	
	pidNotifier->StartPeriodic(units::millisecond_t(20));
	ntNotifier->StartPeriodic(ntUpdateRate);
}

void SwerveModule::setState(double angle, double motorSetpoint, MotorControlMode wheelControlMode) {
	steerController->SetSetpoint(angle);
	wheelMotor->set(motorSetpoint, wheelControlMode);
	
	controlMode = wheelControlMode;
	wheelSetpoint = motorSetpoint;
	steerSetpoint = angle;
}

SwerveWheel SwerveModule::getState() const {
	SwerveWheel out;
	out.wheelVelocity = (wheelMotor->getVelocity() / (2.0 * M_PI)) / gearRatio * wheelCircumference;
	out.wheelPosition = (wheelMotor->getPosition() / (2.0 * M_PI)) / gearRatio * wheelCircumference;
	out.wheelAngle = -steerMotor->getPosition();
	out.wheelAngularVelocity = -steerMotor->getVelocity();
	
	return out;
}

void SwerveModule::updateNT() {
	auto state = getState();
	
	velocityEntry.SetDouble(state.wheelVelocity);
	angleEntry.SetDouble(state.wheelAngle);
	wheelSetpointEntry.SetDouble(wheelSetpoint);
	steerSetpointEntry.SetDouble(steerSetpoint);
	controlModeEntry.SetString(toString(controlMode));
	encoderState.SetDouble(wheelMotor->getPosition());
	
	auto analogPos = steerMotor->getPosition();
	analogPos += analogOffset;
	analogPos = EctoMath::wrapAngle(analogPos);
	rawAnalog.SetDouble(analogPos);
}

