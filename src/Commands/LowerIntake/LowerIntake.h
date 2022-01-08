//
// Created by tener on 10/12/21.
//

#ifndef BOTBUSTERS_REBIRTH_LOWERINTAKE_H
#define BOTBUSTERS_REBIRTH_LOWERINTAKE_H

#include <frc2/command/CommandHelper.h>
#include <frc2/command/CommandBase.h>
#include "Systems/Intake/Intake.h"


class LowerIntake : public frc2::CommandHelper<frc2::CommandBase, LowerIntake> {
public:
	
	LowerIntake(const std::shared_ptr<Intake> &intake, double targetPct = 1.00);
	
	void Initialize() override;
	
	void Execute() override;
	
	void End(bool interrupted) override;
	
	bool IsFinished() override;

private:
	std::shared_ptr<Intake> intake;
	double targetPct;
};

#endif //BOTBUSTERS_REBIRTH_LOWERINTAKE_H
