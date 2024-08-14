#pragma once
#ifndef __GOAL_HPP
#define __GOAL_HPP

#include "common_include.h"

VelocityVector GetGoalVelocity(PositionVector pose, PositionVector goal);
VelocityVector GetGoalVelocity(PositionVector pose, PositionVector goal, uint8_t FId, FormationName Fna);
double SquaredNorm(vector<double> num);
void NormaliseVelocity(VelocityVector& vec);

#endif
