//
// Created by abiel on 2/18/20.
//

#include "PreloadBall.h"

PreloadBall::PreloadBall(const std::shared_ptr<BallFeeder> &feeder, double preloadSpeed) {
	this->feeder = feeder;
	this->preloadSpeed = preloadSpeed;
}

void PreloadBall::Initialize() {
	feeder->stopBallAtSwitch(true);
	feeder->feedBalls(preloadSpeed);
}

void PreloadBall::Execute() {
	;
}

bool PreloadBall::IsFinished() {
	return feeder->isBallPreloaded();
}

void PreloadBall::End(bool interrupted) {
	feeder->stopFeeding();
}