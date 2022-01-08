#pragma once

#include <frc/geometry/Translation2d.h>
#include <frc/geometry/Rotation2d.h>
#include <fmt/format.h>
#include <functional>
#include <vector>
#include <cmath>
#include <Core/VisionManager/VisionManager.h>


struct PathFollowerRotationProfileEntry {
	PathFollowerRotationProfileEntry(double x, double y, double range) {
		this->tX = x;
		this->tY = y;
		this->range = range;
	}
	
	virtual bool shouldActivate(double x, double y) {
		bool inRangeOut = PathFollowerRotationProfileEntry::inRange(x, y);
		bool res = !hasSet && inRangeOut;
		if (res) {
			hasSet = true;
			
		}
		return res;
	}
	
	virtual bool inRange(double x, double y) {
		if (std::hypot(x - tX, y - tY) < range) {
			return true;
		} else return false;
	}
	
	virtual frc::Rotation2d getTargetHeading() = 0;
	
	double tX, tY;
	bool hasSet{false};
	double range;
};

struct PathFollowerRotationThetaEntry : public PathFollowerRotationProfileEntry {
	PathFollowerRotationThetaEntry(double x, double y, double range, double theta) : PathFollowerRotationProfileEntry(x,
	                                                                                                                  y,
	                                                                                                                  range) {
		this->theta = theta;
	}
	
	
	frc::Rotation2d getTargetHeading() override {
		return {units::radian_t(theta)};
		
	}
	
	double theta;
};

struct PathFollowerRotationStationaryEntry : public PathFollowerRotationProfileEntry {
	PathFollowerRotationStationaryEntry(double x, double y, double range, double tX, double tY)
			: PathFollowerRotationProfileEntry(x, y, range) {
		this->targetX = tX;
		this->targetY = tY;
	}
	
	
	bool inRange(double x, double y) override {
		rX = x;
		rY = y;
		return PathFollowerRotationProfileEntry::inRange(x, y);
	}
	
	frc::Rotation2d getTargetHeading() override {
		double dX = targetX - rX;
		double dY = targetY - rY;
		
		double theta = std::atan2(dY, dX);
		frc::SmartDashboard::PutNumber("print Theta/ Theta", theta);
		return {units::radian_t(-theta)};
	}
	
	double rX = 0, rY = 0;
	double targetX = 0, targetY = 0;
};

struct PathFollowerRotationVisionEntry : public PathFollowerRotationProfileEntry {
	PathFollowerRotationVisionEntry(double x, double y, double range, double disableRange,
	                                const std::shared_ptr<EctoSwerve> &swerve,
	                                const std::shared_ptr<VisionManager> &visionManager)
			: PathFollowerRotationProfileEntry(x, y, range) {
		this->visionManager = visionManager;
		this->swerve = swerve;
		this->disableRange = disableRange;
		visionTable = ntInstance.GetTable("photonvision/gloworm");
		visionAngle = visionTable->GetEntry("targetYaw");
		hasTarget = visionTable->GetEntry("hasTarget");
	}
	
	bool shouldActivate(double x, double y) override {
		bool inRangeOut = inRange(x, y);
		if (PathFollowerRotationProfileEntry::shouldActivate(x, y)) {
			return true;
		}
		
		if (!hasActivated) {
			if (hasSet && std::hypot(x - tX, y - tY) > disableRange) {
				hasActivated = true;
				return false;
			} else if (hasSet && std::hypot(x - tX, y - tY) < disableRange) {
				return true;
			}
		}
		
		
		return false;
	}
	
	
	frc::Rotation2d getTargetHeading() override {
		if (hasTarget.GetBoolean(false)) {
			state = -visionAngle.GetDouble(0);
			frc::SmartDashboard::PutNumber("cameraYawConstant", state);
			swerveAngle = swerve->getPose().Rotation();
			angle = frc::Rotation2d(units::degree_t(state));
		} else {
			state = 0;
		}
		return angle + swerveAngle;
	}
	
	std::shared_ptr<EctoSwerve> swerve;
	std::shared_ptr<VisionManager> visionManager;
	
	
	nt::NetworkTableInstance ntInstance = nt::NetworkTableInstance::GetDefault();
	std::shared_ptr<nt::NetworkTable> visionTable;
	nt::NetworkTableEntry visionAngle, hasTarget;
	
	frc::Rotation2d angle;
	frc::Rotation2d swerveAngle;
	
	
	double state = std::numeric_limits<double>::max();
	double disableRange = 0;
	bool hasActivated = false;
};


class PathFollowerRotationProfile {
public:
	explicit PathFollowerRotationProfile(
			const std::vector<std::shared_ptr<PathFollowerRotationProfileEntry>> &entries) {
		this->entries = entries;
	}
	
	PathFollowerRotationProfile() {
		;
	}
	
	std::function<frc::Rotation2d(double x, double y)> getFun() {
		return [this](const double x, const double y) {
			for (const auto &entry: entries) {
				if (entry->shouldActivate(x, y)) {
					lastSetpoint = entry->getTargetHeading();
					return lastSetpoint;
				}
			}
			return lastSetpoint;
		};
	}

private:
	frc::Rotation2d lastSetpoint;
	std::vector<std::shared_ptr<PathFollowerRotationProfileEntry>> entries;
};
