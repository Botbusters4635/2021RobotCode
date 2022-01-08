//
// Created by Abiel on 12/9/18.
//

#ifndef BOTBUSTERSREBIRTH_WATCHDOG_H
#define BOTBUSTERSREBIRTH_WATCHDOG_H

#include <thread>
#include <functional>
#include <chrono>
#include <atomic>
#include <future>

using duration = std::chrono::milliseconds;

class EctoWatchdog {
public:
	EctoWatchdog(const std::string &threadName, duration timeout, duration timeBetweenCalls,
	             std::function<void()> callbackIn);
	
	void start();
	
	void stop();
	
	bool isRunning() const;

private:
	std::string threadName;
	
	duration timeout;
	duration timeBetweenCalls;
	
	std::function<void()> callback;
	
	std::thread watchdogThread;
	
	std::atomic<bool> running;
	
	void loop();
};


#endif //BOTBUSTERSREBIRTH_WATCHDOG_H
