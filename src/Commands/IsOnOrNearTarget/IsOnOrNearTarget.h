//
// Created by cc on 23/11/21.
//

#ifndef BOTBUSTERS_REBIRTH_ISONORNWARTARGET_CPP_H
#define BOTBUSTERS_REBIRTH_ISONORNWARTARGET_CPP_H

#include   "Systems/EctoSwerve/EctoSwerve.h"

#include <frc2/command/CommandHelper.h>
#include <frc2/command/CommandBase.h>
#include <functional>
#include <functional>
#include <vector>
#include <cmath>

class IsOnOrNearTarget : public frc2::CommandHelper<frc2::CommandBase, IsOnOrNearTarget> {
public:
	IsOnOrNearTarget(const std::shared_ptr<EctoSwerve> &swerve, double _tX, double _tY, double _tol = 0.3);
	
	void Initialize() override;
	
	void Execute() override;
	
	void End(bool interrupted) override;
	
	bool IsFinished() override;

private:
	std::shared_ptr<EctoSwerve> swerve;
	double tX, tY;
	double tol;
	bool check;
};

#endif //BOTBUSTERS_REBIRTH_ISONORNWARTARGET_CPP_H
