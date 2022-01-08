//
// Created by abiel on 2/11/20.
//

#ifndef BOTBUSTERSREBIRTH_FEEDBALLS_H
#define BOTBUSTERSREBIRTH_FEEDBALLS_H

#include <frc2/command/CommandHelper.h>
#include <frc2/command/CommandBase.h>

#include "Systems/BallFeeder/BallFeeder.h"

enum class FeedBallsState {
	Feeding,
	Stuck,
	Finished
};

class FeedBalls : public frc2::CommandHelper<frc2::CommandBase, FeedBalls> {
public:
	FeedBalls(const std::shared_ptr<BallFeeder> &feeder, int ballsToFeed, double timeout = 10);
	
	void Initialize() override;
	
	void Execute() override;
	
	void End(bool interrupted) override;
	
	bool IsFinished() override;

private:
	std::shared_ptr<BallFeeder> feeder;
	
	FeedBallsState state;
	
	int ballsToFeed;
	
	const double feedSpeed = -1.0;
	const double unstickSpeed = 1;
	
	const double expectedTimePerBall = 2.1;
	
	double stuckStartTime;
	const double unstickTime = 0.25;
	
	double startTime;
	
	double lastBallCountChange;
	int lastBallCount;
	
	double lastTime;
	double timeSpentFeeding{0};
	
	double timeout;
	
	int initialBallCount;
	
	bool hasInitialized{false};
};


#endif //BOTBUSTERSREBIRTH_FEEDBALLS_H
