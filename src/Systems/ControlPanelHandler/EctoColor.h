//
// Created by karen on 07/01/2020.
//

#ifndef BOTBUSTERS_REBIRTH_COLOR_H
#define BOTBUSTERS_REBIRTH_COLOR_H

#include <iostream>

enum class EctoColor {
	Yellow,
	Red,
	Green,
	Blue,
	Error
};

size_t colorToPosition(EctoColor color);

EctoColor positionToColor(size_t position);

EctoColor offsetColor(EctoColor color, size_t offset);

size_t operator-(EctoColor a, EctoColor b);

#endif