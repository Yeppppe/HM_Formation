#include "../include/formation_controller.h"

double GoalW = .2;
double FormationW = .2;
double AvoidObsW = .7;
double NoiseW = .05;
double OverallWeight = 1.85;

// Controller for Leader
LinearAngularVector GetControllerOutput(PositionVector CurPos, 
PositionVector GoalPos, const LaserVector& CurLaser)
{
    LinearAngularVector ControllerOutput = {0, 0};
    VelocityVector GoalVelocity = GetGoalVelocity(CurPos, GoalPos);
    VelocityVector AvoidObsVelocity = GetAvoidVelocity(CurLaser, CurPos[YAW]);
    VelocityVector WeightVelocity = {0, 0};

    WeightVelocity[UX] = (GoalVelocity[UX] * GoalW  + AvoidObsVelocity[UX] * AvoidObsW) * OverallWeight;
    WeightVelocity[UY] = (GoalVelocity[UY] * GoalW  + AvoidObsVelocity[UY] * AvoidObsW) * OverallWeight;

    ControllerOutput = FeedbackLinearized(WeightVelocity, CurPos[YAW], EPSILON);

    // std::cout << "goal:" << GoalVelocity[0] << ", " << GoalVelocity[1] << std::endl;
    // std::cout << "AvoidObsVelocity:" << AvoidObsVelocity[0] << ", " << AvoidObsVelocity[1] << std::endl;
    // std::cout << "ControllerOutput:" << ControllerOutput[0] << ", " << ControllerOutput[1] << std::endl;
    // std::cout << "laser : " << CurLaser[0] << ", " << CurLaser[1] << ", " << CurLaser[2] << ", " << CurLaser[3]
    //     << ", " << CurLaser[4] << std::endl;

    ControllerOutput[UV] = ControllerOutput[UV] * 0.5;
    ControllerOutput[UW] = ControllerOutput[UW] * 0.5;

    return ControllerOutput;
}

// Controller for Follower
LinearAngularVector GetControllerOutput(PositionVector LPos, PositionVector CurPos, 
    uint8_t CurId, FormationName CurFor, const LaserVector& CurLaser, double* Err)
{
    LinearAngularVector ControllerOutput = {0, 0};
    VelocityVector FormationVelocity = GetFormationVelocity(LPos, CurPos, CurId, CurFor, LEADER, Err);
    VelocityVector AvoidObsVelocity = GetAvoidVelocity(CurLaser, CurPos[YAW]);
    VelocityVector WeightVelocity = {0, 0};
    
    WeightVelocity[UX] = (FormationVelocity[UX] * FormationW  + AvoidObsVelocity[UX] * AvoidObsW) * OverallWeight;
    WeightVelocity[UY] = (FormationVelocity[UY] * FormationW  + AvoidObsVelocity[UY] * AvoidObsW) * OverallWeight;

    ControllerOutput = FeedbackLinearized(WeightVelocity, CurPos[YAW], EPSILON);

    // std::cout << "\n\nID: " << FId << "\n";
    // std::cout << "GetFormationVelocity:" << FormationVelocity[0] << ", " << FormationVelocity[1] << std::endl;
    // std::cout << "WeightVelocity:" << WeightVelocity[0] << ", " << WeightVelocity[1] << std::endl;
    // std::cout << "ControllerOutput:" << ControllerOutput[0] << ", " << ControllerOutput[1] << std::endl;

    return ControllerOutput;
}

// Controller for Center
LinearAngularVector GetControllerOutput(
    PositionVector LPos, PositionVector CurPos, 
    PositionVector GoalPos, uint8_t CurId, FormationName CurFor, 
    const LaserVector& CurLaser, double* Err)
{
    LinearAngularVector ControllerOutput = {0, 0};
    VelocityVector GoalVelocity = GetGoalVelocity(CurPos, GoalPos, CurId, CurFor);
    VelocityVector FormationVelocity = GetFormationVelocity(LPos, CurPos, CurId, CurFor, CENTER, Err);
    VelocityVector AvoidObsVelocity = GetAvoidVelocity(CurLaser, CurPos[YAW]);
    VelocityVector WeightVelocity = {0, 0};

    WeightVelocity[UX] = (GoalVelocity[UX] * GoalW + FormationVelocity[UX] * FormationW + 
        AvoidObsVelocity[UX] * AvoidObsW) * OverallWeight * 0.7;
    WeightVelocity[UY] = (GoalVelocity[UY] * GoalW  + FormationVelocity[UY] * FormationW + 
        AvoidObsVelocity[UY] * AvoidObsW) * OverallWeight * 0.7;
    
    ControllerOutput = FeedbackLinearized(WeightVelocity, CurPos[YAW], EPSILON);

    return ControllerOutput;
}

// Controller for Neighbor
LinearAngularVector GetControllerOutput(
    vector<PositionVector>& RobotPos, vector<vector<int>>& Topo, 
    uint8_t CurId, PositionVector GoalPos, FormationName CurFor, 
    const LaserVector& CurLaser, double* Err)
{
    PositionVector CurPos = RobotPos[CurId];
    LinearAngularVector ControllerOutput = {0, 0};
    VelocityVector GoalVelocity = GetGoalVelocity(CurPos, GoalPos, CurId, CurFor);
    VelocityVector FormationVelocity = GetFormationVelocity(RobotPos, Topo, CurId, CurFor, Err);
    VelocityVector AvoidObsVelocity = GetAvoidVelocity(CurLaser, CurPos[YAW]);
    VelocityVector WeightVelocity = {0, 0};
    
    WeightVelocity[UX] = (GoalVelocity[UX] * GoalW + FormationVelocity[UX] * FormationW + 
        AvoidObsVelocity[UX] * AvoidObsW) * OverallWeight * 0.7;
    WeightVelocity[UY] = (GoalVelocity[UY] * GoalW  + FormationVelocity[UY] * FormationW + 
        AvoidObsVelocity[UY] * AvoidObsW) * OverallWeight * 0.7;
    
    ControllerOutput = FeedbackLinearized(WeightVelocity, CurPos[YAW], EPSILON);

    return ControllerOutput;
}

LinearAngularVector FeedbackLinearized(VelocityVector velocity, double yaw, double epsilon)
{
    LinearAngularVector ret = {0, 0};
    double VeloThreshold = 0.01;

    if (SquaredNorm(velocity) < VeloThreshold) {
        return ret;
    }

    ret[UV] = velocity[UX] * cos(yaw) + velocity[UY] * sin(yaw);
    ret[UW] = (velocity[UY] * cos(yaw) - velocity[UX] * sin(yaw)) / epsilon;
    if (ret[UV] < -MAXLINEARSPEED) {
        ret[UV] = -MAXLINEARSPEED;
    } else if (ret[UV] > MAXLINEARSPEED) {
        ret[UV] = MAXLINEARSPEED;
    }
    if (ret[UW] > MAXANGULARSPEED) {
        ret[UW] = MAXANGULARSPEED;
    } else if (ret[UW] < -MAXANGULARSPEED) {
        ret[UW] = -MAXANGULARSPEED;
    }
    
    return ret;
}

