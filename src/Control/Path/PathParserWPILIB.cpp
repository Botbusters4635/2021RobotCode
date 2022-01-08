//
// Created by abiel on 1/2/20.
//

//Parses wpilib paths from PathParser (2020)

#include "Control/Path/PathParserWPILIB.h"
#include "Control/Path/Path.h"
#include <wpi/json.h>
#include <iostream>
#include <fstream>

Path PathParserWPILIB::loadPath(const std::string &filePath) {
	std::ifstream file(filePath);
	
	if (!file.good()) {
		throw std::runtime_error("Invalid file!");
	}
	
	//TODO
	throw std::runtime_error("NOT IMPLEMENTED");
}