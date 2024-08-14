#pragma once
#ifndef __FORMATION_HPP
#define __FORMATION_HPP

#include "common_include.h"

VelocityVector GetFormationVelocity(PositionVector LPos, PositionVector FPos, 
    uint8_t FId, FormationName Fna, FormationMode Fmode, double* Err = nullptr);

VelocityVector GetFormationVelocity(vector<PositionVector>& RobotPos,
    vector<vector<int>>& Topo, uint8_t FId, FormationName Fna, double* Err = nullptr);

#endif
