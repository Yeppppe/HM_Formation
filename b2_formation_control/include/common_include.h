#pragma once
#ifndef __COMMON_INCLUDE_H
#define __COMMON_INCLUDE_H

#include <iostream>
#include <cmath>
#include <cstdint>
#include <vector>
#include <string>
using std::vector;
using std::string;

constexpr int    CONTROLRATE = 20;
constexpr double DEADZONE = 0.03;
constexpr double CONTROLZONE = 0.20;
constexpr double MAXLINEARSPEED = 1.0;
constexpr double MAXANGULARSPEED = 1.0;
constexpr double EPSILON = 0.2;
constexpr uint8_t X = 0;
constexpr uint8_t Y = 1;
constexpr uint8_t YAW = 2;
constexpr uint8_t UX = 0;
constexpr uint8_t UY = 1;
constexpr uint8_t UV = 0;       // Linear
constexpr uint8_t UW = 1;       // Angular
enum LaserDirction {FRONT, LEFTFRONT, RIGHTFRONT, LEFT, RIGHT};
enum FormationName {LINE, COLUMN, DIAMOND, WEDGE};
enum FormationMode {LEADER, CENTER, NEIGHBOR};

typedef vector<double> PositionVector;      // X, Y
typedef vector<double> VelocityVector;      // UX, UY
typedef vector<double> LinearAngularVector; // UV, UW
typedef vector<double> LaserVector;         // LEFT, LEFTFRONT, FRONT, RIGHTFRONT, RIGHT

// Formation spacing parameter for the formation
constexpr double ROBOT_RADIUS = 0.105 / 2.;
constexpr double SPACING_DIST = 0.60 + ROBOT_RADIUS;
constexpr bool ALLOW_FORMATION_ROTATION = false;

// LINE  3 1 0 2 4
// COLUMN   0
//          1
//          2
//          3
//          4
// DIAMOND      0
//          3   1   2
//              4
// WEDGE        0   
//          1       2
//      3               4
const vector<vector<PositionVector>> FormationBias = {
    {{0, 0}, {-SPACING_DIST, 0}, {SPACING_DIST, 0}, {-2 * SPACING_DIST, 0}, {2 * SPACING_DIST, 0}},
    {{0, 0}, {0, -1 * SPACING_DIST}, {0, -2 * SPACING_DIST}, {0, -3 * SPACING_DIST}, {0, -4 * SPACING_DIST}},
    {{0, 0}, {0, -SPACING_DIST}, {SPACING_DIST, -SPACING_DIST}, {-SPACING_DIST, -SPACING_DIST}, {0, -2 * SPACING_DIST}},
    {{0, 0}, {-SPACING_DIST, -SPACING_DIST}, {SPACING_DIST, -SPACING_DIST}, {-2 * SPACING_DIST, -2 * SPACING_DIST}, {2 * SPACING_DIST, -2 * SPACING_DIST}}
};

const vector<vector<PositionVector>> FormationBias_01 = {
    {{0, 0}, {-SPACING_DIST, 0}, {SPACING_DIST, 0}, {-2 * SPACING_DIST, 0}, {2 * SPACING_DIST, 0}},
    {{0, 2 * SPACING_DIST}, {0, SPACING_DIST}, {0, 0}, {0, -1 * SPACING_DIST}, {0, -2 * SPACING_DIST}},
    {{0, SPACING_DIST}, {0, 0}, {SPACING_DIST, 0}, {-SPACING_DIST, 0}, {0, -1 * SPACING_DIST}},
    {{0, SPACING_DIST}, {-SPACING_DIST, 0}, {SPACING_DIST, 0}, {-2 * SPACING_DIST, -1 * SPACING_DIST}, {2 * SPACING_DIST, -1 * SPACING_DIST}}
};

const vector<vector<vector<PositionVector>>> FormationMatrix = {
    {
        {
            {{0,0},{SPACING_DIST,0},{-SPACING_DIST,0},{2*SPACING_DIST,0},{-2*SPACING_DIST,0}},
            {{-SPACING_DIST,0},{0,0},{-2*SPACING_DIST,0},{SPACING_DIST,0},{-3*SPACING_DIST,0}},
            {{SPACING_DIST,0},{2*SPACING_DIST,0},{0,0},{3*SPACING_DIST,0},{-SPACING_DIST,0}},
            {{-2*SPACING_DIST,0},{-SPACING_DIST,0},{-3*SPACING_DIST,0},{0,0},{-4*SPACING_DIST}},
            {{2*SPACING_DIST,0},{3*SPACING_DIST,0},{SPACING_DIST,0},{4*SPACING_DIST,0},{0,0}}
        },
        {
            {{0,0},{0,SPACING_DIST},{0,2*SPACING_DIST},{0,3*SPACING_DIST},{0,4*SPACING_DIST}},
            {{0,-SPACING_DIST},{0,0},{0,SPACING_DIST},{0,2*SPACING_DIST},{0,3*SPACING_DIST}},
            {{0,-2*SPACING_DIST},{0,-SPACING_DIST},{0,0},{0,SPACING_DIST},{0,2*SPACING_DIST}},
            {{0,-3*SPACING_DIST},{0,-2*SPACING_DIST},{0,-SPACING_DIST},{0,0},{0,SPACING_DIST}},
            {{0,-4*SPACING_DIST},{0,-3*SPACING_DIST},{0,-2*SPACING_DIST},{0,-SPACING_DIST},{0,0}}
        },
        {
            {{0,0},{0,SPACING_DIST},{-SPACING_DIST,SPACING_DIST},{SPACING_DIST,SPACING_DIST},{0,2*SPACING_DIST}},
            {{0,-SPACING_DIST},{0,0},{-SPACING_DIST,0},{SPACING_DIST,0},{0,SPACING_DIST}},
            {{SPACING_DIST,-SPACING_DIST},{SPACING_DIST,0},{0,0},{2*SPACING_DIST,0},{SPACING_DIST,SPACING_DIST}},
            {{-SPACING_DIST,-SPACING_DIST},{-SPACING_DIST,0},{-2*SPACING_DIST,0},{0,0},{-SPACING_DIST,SPACING_DIST}},
            {{0,-2*SPACING_DIST},{0,-SPACING_DIST},{-SPACING_DIST,-SPACING_DIST},{SPACING_DIST,-SPACING_DIST},{0,0}}
        },
        {
            {{0,0},{SPACING_DIST,SPACING_DIST},{-SPACING_DIST,SPACING_DIST},{2*SPACING_DIST,2*SPACING_DIST},{-2*SPACING_DIST,2*SPACING_DIST}},
            {{-SPACING_DIST,-SPACING_DIST},{0,0},{-2*SPACING_DIST,0},{SPACING_DIST,SPACING_DIST},{-3*SPACING_DIST,SPACING_DIST}},
            {{SPACING_DIST,-SPACING_DIST},{2*SPACING_DIST,0},{0,0},{3*SPACING_DIST,SPACING_DIST},{-SPACING_DIST,SPACING_DIST}},
            {{-2*SPACING_DIST,-2*SPACING_DIST},{-SPACING_DIST,-SPACING_DIST},{-3*SPACING_DIST,-SPACING_DIST},{0,0},{-4*SPACING_DIST,0}},
            {{2*SPACING_DIST,-2*SPACING_DIST},{3*SPACING_DIST,-SPACING_DIST},{SPACING_DIST,-SPACING_DIST},{4*SPACING_DIST,0},{0,0}}
        }
    }
};

#endif
