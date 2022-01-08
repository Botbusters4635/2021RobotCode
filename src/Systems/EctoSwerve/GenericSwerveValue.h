//
// Created by abiel on 7/14/20.
//

#ifndef BOTBUSTERSREBIRTH_GENERICSWERVEVALUE_H
#define BOTBUSTERSREBIRTH_GENERICSWERVEVALUE_H

#include <networktables/NetworkTable.h>
#include <networktables/NetworkTableEntry.h>
#include <networktables/NetworkTableInstance.h>

class GenericSwerveValue {
public:
	double backLeft{0};
	double backRight{0};
	
	double frontLeft{0};
	double frontRight{0};
	
	static GenericSwerveValue min(const GenericSwerveValue &first, const GenericSwerveValue &second) {
		GenericSwerveValue out;
		out.frontLeft = std::min(first.frontLeft, second.frontLeft);
		out.frontRight = std::min(first.frontRight, second.frontRight);
		out.backLeft = std::min(first.backLeft, second.backLeft);
		out.backRight = std::min(first.backRight, second.backRight);
		
		return out;
	}
	
	static GenericSwerveValue max(const GenericSwerveValue &first, const GenericSwerveValue &second) {
		GenericSwerveValue out;
		out.frontLeft = std::max(first.frontLeft, second.frontLeft);
		out.frontRight = std::max(first.frontRight, second.frontRight);
		out.backLeft = std::max(first.backLeft, second.backLeft);
		out.backRight = std::max(first.backRight, second.backRight);
		
		return out;
	}
	
	static GenericSwerveValue avg(const GenericSwerveValue &first, const GenericSwerveValue &second) {
		GenericSwerveValue out;
		out.frontLeft = (first.frontLeft + second.frontLeft) / 2.0;
		out.frontRight = (first.frontRight + second.frontRight) / 2.0;
		out.backLeft = (first.backLeft + second.backLeft) / 2.0;
		out.backRight = (first.backRight + second.backRight) / 2.0;
		
		return out;
	}
	
	void publishToNT(const std::shared_ptr<nt::NetworkTable> &table, const std::string &entryName) {
		table->GetEntry(entryName + "/frontLeftWheel").SetDouble(frontLeft);
		table->GetEntry(entryName + "/frontRightWheel").SetDouble(frontRight);
		table->GetEntry(entryName + "/backLeftWheel").SetDouble(backLeft);
		table->GetEntry(entryName + "/backRightWheel").SetDouble(backRight);
	}
};

#endif //BOTBUSTERSREBIRTH_GENERICSWERVEVALUE_H
