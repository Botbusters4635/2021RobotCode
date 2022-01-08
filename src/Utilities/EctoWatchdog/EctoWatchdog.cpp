#include "EctoWatchdog.h"

EctoWatchdog::EctoWatchdog(const std::string &threadName, duration
timeout,
                           duration timeBetweenCalls,
                           std::function<void()> callback) {
	this->threadName = threadName;
	this->timeout = timeout;
	this->timeBetweenCalls = timeBetweenCalls;
	this->callback = callback;
	
	if (timeout <= timeBetweenCalls)
		throw std::logic_error("Timeout smaller than the time between calls!");
	
	//running.store(true);
	//watchdogThread = std::thread(&Watchdog::Loop, this);
}

void EctoWatchdog::start() {
	running.store(true);
	watchdogThread = std::thread(&EctoWatchdog::loop, this);
	pthread_setname_np(watchdogThread.native_handle(), threadName.c_str());
}

void EctoWatchdog::stop() {
	running.store(false);
	watchdogThread.join();
}

bool EctoWatchdog::isRunning() const {
	return running.load();
}

void EctoWatchdog::loop() {
	while (running.load()) {
		//std::chrono::system_clock::time_point startTime = std::chrono::system_clock::now();
		
		//Starts to run callback
		auto future = std::async(std::launch::async, callback);
		
		std::this_thread::sleep_for(timeBetweenCalls);
		
		auto status = future.wait_for(timeout);
		
		if (status == std::future_status::timeout) {
			throw std::runtime_error("Function timed out! Aborting!");
		}
	}
}
