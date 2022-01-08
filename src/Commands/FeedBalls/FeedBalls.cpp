//
// Created by abiel on 2/11/20.
//

#include "FeedBalls.h"

#include <frc/Timer.h>

FeedBalls::FeedBalls(const std::shared_ptr<BallFeeder> &feeder, int ballsToFeed, double timeout) {
	this->feeder = feeder;
	this->ballsToFeed = ballsToFeed;
	lastBallCount = feeder->getBallCount();
	
	this->timeout = timeout;
	
	state = FeedBallsState::Feeding;
	
	this->SetName("FeedBalls");
}

void FeedBalls::Initialize() {
	startTime = frc::Timer::GetFPGATimestamp();
	lastBallCountChange = startTime;
	stuckStartTime = startTime;
	
	feeder->stopBallAtSwitch(false);
	initialBallCount = feeder->getBallCount();
	
	hasInitialized = true;
}

void FeedBalls::Execute() {
	const auto updatedBallCount = feeder->getBallCount() - initialBallCount;
	
	if (updatedBallCount >= ballsToFeed) {
		//Finished
		state = FeedBallsState::Finished;
	}
	
	std::cout << updatedBallCount << std::endl;
	
	switch (state) {
		case FeedBallsState::Feeding:
			if (updatedBallCount != lastBallCount) {
				//Ball count changed
				lastBallCountChange = frc::Timer::GetFPGATimestamp();
				lastBallCount = updatedBallCount;
			}
			
			if (frc::Timer::GetFPGATimestamp() - lastBallCountChange >= expectedTimePerBall) {
				//Ball not fed, stuck
				state = FeedBallsState::Stuck;
				stuckStartTime = frc::Timer::GetFPGATimestamp();
				break;
			}
			
			//std::cout << "Fed: " << updatedBallCount << std::endl;
			
			feeder->feedBalls(feedSpeed);
			
			timeSpentFeeding += (frc::Timer::GetFPGATimestamp() - lastTime);
			break;
		
		case FeedBallsState::Stuck:
			//std::cout << "Stuck" << std::endl;
			
			if (frc::Timer::GetFPGATimestamp() - stuckStartTime >= unstickTime) {
				//Finished "unsticking" the ball
				state = FeedBallsState::Feeding;
				lastBallCountChange = frc::Timer::GetFPGATimestamp();
				break;
			}
			
			feeder->feedBalls(unstickSpeed);
			
			break;
		
		case FeedBallsState::Finished:
			feeder->stopFeeding();
			break;
	}
	
	lastTime = frc::Timer::GetFPGATimestamp();
}

bool FeedBalls::IsFinished() {
	if (timeSpentFeeding > timeout) {
		//Timeout
		//return true;
	}
	
	return hasInitialized and (feeder->getBallCount() - initialBallCount) >= ballsToFeed;
}

void FeedBalls::End(bool interrupted) {
	feeder->stopFeeding();
}