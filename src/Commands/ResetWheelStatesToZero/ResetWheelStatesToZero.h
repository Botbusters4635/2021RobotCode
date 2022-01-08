//
// Created by cc on 06/01/22.
//

#ifndef BOTBUSTERS_REBIRTH_RESETWHEELSTATESTOZERO_H
#define BOTBUSTERS_REBIRTH_RESETWHEELSTATESTOZERO_H

#include <frc2/command/CommandHelper.h>
#include <frc2/command/CommandBase.h>
#include "Systems/EctoSwerve/EctoSwerve.h"

class ResetWheelStateToZero : public frc2::CommandHelper<frc2::CommandBase, ResetWheelStateToZero> {
public:
	ResetWheelStateToZero(const std::shared_ptr<EctoSwerve> &swerve);
	
	void Initialize() override;
	
	void Execute() override;
	
	void End(bool interrupted) override;
	
	bool IsFinished() override;

private:
	std::shared_ptr<EctoSwerve> swerve;
	
	
};

#endif //BOTBUSTERS_REBIRTH_RESETWHEELSTATESTOZERO_H
