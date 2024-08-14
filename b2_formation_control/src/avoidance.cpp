#include "../include/avoidance.h"

// based-on rules
VelocityVector GetAvoidVelocity(const LaserVector& LVec, double yaw)
{
    int                 FrontObstacleSide = 1;      // 1:正前方障碍物靠右，-1：靠左
    LaserVector         SensorsVec = LVec;
    VelocityVector      VRet(2);
    LinearAngularVector VTmp = {0.0, 0.0};
    
    if (SensorsVec[FRONT] < 0) {
        SensorsVec[FRONT] = -SensorsVec[FRONT];
        FrontObstacleSide = -1;
    }
    for (double& _s : SensorsVec) {
        _s = tanh(_s);
    }
    
    // if the right hand side detects an approaching object , alter w to move left
    if (SensorsVec[RIGHTFRONT] < tanh(0.4)) {
        // std::cout << "fire fornt right!" << std::endl;
        VTmp[UV] -= 0.05; 
        VTmp[UW] += (1.5 * (1.3 - SensorsVec[RIGHTFRONT]));
    }

    // if the left hand side detects and approaching object, alter w to move to the right
    if (SensorsVec[LEFTFRONT] < tanh(0.4)) {
        // std::cout << "fire fornt left!" << std::endl;
        VTmp[UV] -= 0.05; 
        VTmp[UW] -= (1.5 * (1.3 - SensorsVec[LEFTFRONT]));
    }

    // if robot is very close to the right hand slide, adjust left a little
    if (SensorsVec[RIGHT] < tanh(0.2)) {
        // std::cout << "fire right!" << std::endl;
        VTmp[UW] += (0.3 * (1.0 - SensorsVec[RIGHT]));
    }

    // // if robot is very close to the left hand slide, adjust right a little
    if (SensorsVec[LEFT] < tanh(0.2)) {
        // std::cout << "fire left!" << std::endl;
        VTmp[UW] -= (0.3 * (1.0 - SensorsVec[LEFT]));
    }

    // if close to front, move away
    if (SensorsVec[FRONT] < tanh(0.4)) {
        // std::cout << "fire fornt!" << std::endl;
        VTmp[UV] -= 0.2; 
        VTmp[UW] += (FrontObstacleSide * 4.0 * (1.0 - 0.5 * SensorsVec[FRONT]));
    }
    
    VRet[UX] = VTmp[UV] * cos(yaw) - EPSILON * VTmp[UW] * sin(yaw);
    VRet[UY] = VTmp[UV] * sin(yaw) + EPSILON * VTmp[UW] * cos(yaw);
    
    return VRet;
}

