#pragma once
#ifndef __B2_UTIL_H
#define __B2_UTIL_H

#include "move_robot.hpp"

bool AllRobotsReady(const vector<MoveRobot*>& robots);
bool IsCorridor(const vector<MoveRobot*>& robots);
bool SpawnTargetModel(ros::NodeHandle* n, PositionVector tar_pos);
bool IsArriveGoal(PositionVector cur, PositionVector goal);
PositionVector GetUnitCenter(const vector<PositionVector>& robot_pos);
vector<vector<int>> GetTopology(string config_path);

#endif

