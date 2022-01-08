//
// Created by cc on 06/01/22.
//

#ifndef BOTBUSTERS_REBIRTH_GOTOANGLE_H
#define BOTBUSTERS_REBIRTH_GOTOANGLE_H

#include <frc2/command/CommandBase.h>
#include <frc2/command/CommandHelper.h>
#include "Systems/EctoSwerve/EctoSwerve.h"

class GoToAngle : public frc2::CommandHelper<frc2::CommandBase, GoToAngle> {
public:
	GoToAngle(const std::shared_ptr<EctoSwerve> &swerve, double angle, double tol);
	
	void Initialize() override;
	
	void Execute() override;
	
	void End(bool interrupted) override;
	
	bool IsFinished() override;

private:
	std::shared_ptr<EctoSwerve> swerve;
	frc::ChassisSpeeds chassisSpeeds;
	frc2::PIDController anglePID{0.9, 0.08, 0.1};
	double state;
	double angle;
	double pidOut;
	double tol;
	
	
};

#endif //BOTBUSTERS_REBIRTH_GOTOANGLE_H
