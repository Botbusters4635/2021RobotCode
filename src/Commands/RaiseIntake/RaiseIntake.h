//
// Created by cc on 6/12/21.
//

#ifndef BOTBUSTERS_REBIRTH_RAISEINTAKE_H
#define BOTBUSTERS_REBIRTH_RAISEINTAKE_H

#include <frc2/command/CommandHelper.h>
#include <frc2/command/CommandBase.h>
#include "Systems/Intake/Intake.h"


class RaiseIntake : public frc2::CommandHelper<frc2::CommandBase, RaiseIntake> {
public:
	
	RaiseIntake(const std::shared_ptr<Intake> &intake, double targetPct = 0);
	
	void Initialize() override;
	
	void Execute() override;
	
	void End(bool interrupted) override;
	
	bool IsFinished() override;

private:
	std::shared_ptr<Intake> intake;
	double targetPct;
};

#endif //BOTBUSTERS_REBIRTH_RAISEINTAKE_H
