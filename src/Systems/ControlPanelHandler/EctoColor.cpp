#include "EctoColor.h"

size_t colorToPosition(EctoColor color) {
	switch (color) {
		case EctoColor::Yellow:
			return 0;
		case EctoColor::Red:
			return 1;
		case EctoColor::Green:
			return 2;
		case EctoColor::Blue:
			return 3;
		default:
			throw std::invalid_argument("Tried to get position of invalid color");
	}
}

EctoColor positionToColor(size_t position) {
	switch (position) {
		case 0:
			return EctoColor::Yellow;
		
		case 1:
			return EctoColor::Red;
		
		case 2:
			return EctoColor::Green;
		
		case 3:
			return EctoColor::Blue;
		
		default:
			throw std::invalid_argument("Tried to get color of invalid position");
	}
}

EctoColor offsetColor(EctoColor color, size_t offset) {
	int pos = (int) colorToPosition(color);
	pos -= offset;
	
	return pos < 0 ? positionToColor(4 - pos) : positionToColor(pos);
}

size_t operator-(EctoColor a, EctoColor b) {
	return colorToPosition(a) - colorToPosition(b);
}

