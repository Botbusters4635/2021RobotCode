//
// Created by abiel on 2/18/20.
//

#ifndef BOTBUSTERSREBIRTH_PRELOADBALL_H
#define BOTBUSTERSREBIRTH_PRELOADBALL_H

#include <frc2/command/CommandHelper.h>
#include <frc2/command/CommandBase.h>

#include "Systems/BallFeeder/BallFeeder.h"

class PreloadBall : public frc2::CommandHelper<frc2::CommandBase, PreloadBall> {
public:
	PreloadBall(const std::shared_ptr<BallFeeder> &feeder, double preloadSpeed = -.45);
	
	void Initialize() override;
	
	void Execute() override;
	
	void End(bool interrupted) override;
	
	bool IsFinished() override;

private:
	std::shared_ptr<BallFeeder> feeder;
	double preloadSpeed;
};


#endif //BOTBUSTERSREBIRTH_PRELOADBALL_H
