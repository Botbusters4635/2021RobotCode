//
// Created by tener on 10/12/21.
//

#ifndef BOTBUSTERS_REBIRTH_USEINTAKE_H
#define BOTBUSTERS_REBIRTH_USEINTAKE_H

#include <frc2/command/CommandHelper.h>
#include <frc2/command/CommandBase.h>
#include "Systems/Intake/Intake.h"

class UseIntake : public frc2::CommandHelper<frc2::CommandBase, UseIntake> {
public:
	
	UseIntake(const std::shared_ptr<Intake> &intake, double TargetPct = 1.00, double state = false);
	
	void Initialize() override;
	
	void Execute() override;
	
	void End(bool interrupted) override;
	
	bool IsFinished() override;

private:
	std::shared_ptr<Intake> intake;
	double targetPct;
	double state;
};

#endif //BOTBUSTERS_REBIRTH_USEINTAKE_H
