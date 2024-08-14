#pragma once
#ifndef __AVOIDANCE_HPP
#define __AVOIDANCE_HPP

#include "common_include.h"

VelocityVector GetAvoidVelocity(const LaserVector& LVec, double yaw);

#endif
