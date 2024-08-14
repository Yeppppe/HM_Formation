#pragma once
#ifndef __FORMATION_CONTROLLER_H
#define __FORMATION_CONTROLLER_H

#include <cmath>

#include "goal.h"
#include "avoidance.h"
#include "formation.h"
#include "common_include.h"


LinearAngularVector GetControllerOutput(PositionVector CurPos, 
    PositionVector GoalPos, const LaserVector& CurLaser);

LinearAngularVector GetControllerOutput(PositionVector LPos, PositionVector FPos, 
    uint8_t FId, FormationName Fna, const LaserVector& CurLaser, double* Err = nullptr);

LinearAngularVector GetControllerOutput(
    PositionVector LPos, PositionVector CurPos, 
    PositionVector GoalPos, uint8_t CurId, FormationName CurFor, 
    const LaserVector& CurLaser, double* Err);

LinearAngularVector GetControllerOutput(
    vector<PositionVector>& RobotPos, vector<vector<int>>& Topo, 
    uint8_t CurId, PositionVector GoalPos, FormationName CurFor, 
    const LaserVector& CurLaser, double* Err);

vector<double> FeedbackLinearized(vector<double> velocity, double yaw, double epsilon);


#endif
