#include "../include/goal.h"

double SquaredNorm(vector<double> num)
{
    double sum = 0.0;

    for (double n : num) {
        sum += pow(n, 2);
    }

    return sqrt(sum);
}

void NormaliseVelocity(VelocityVector& vec)
{
    double n = SquaredNorm(vec);
    if (n < 1e-2) {
        vec[X] = 0;
        vec[Y] = 0;
        return ;
    }
    vec[X] /= n;
    vec[Y] /= n;
}

VelocityVector GetGoalVelocity(PositionVector pose, PositionVector goal)
{
    VelocityVector  GoalVelocity(2, 0);
    PositionVector  Bias(2, 0);
    double          BiasMag = 0.0;

    Bias[X] = goal[X] - pose[X];
    Bias[Y] = goal[Y] - pose[Y];
    BiasMag = SquaredNorm(Bias);
    
    if (BiasMag < DEADZONE) {
        return GoalVelocity;
    }

    GoalVelocity[X] = Bias[X];
    GoalVelocity[Y] = Bias[Y];
    NormaliseVelocity(GoalVelocity);
    if (BiasMag < CONTROLZONE) {
        GoalVelocity[X] = GoalVelocity[X] * sin(M_PI_2 * BiasMag / CONTROLZONE);
        GoalVelocity[Y] = GoalVelocity[Y] * sin(M_PI_2 * BiasMag / CONTROLZONE);
    }

    return GoalVelocity;
}

VelocityVector GetGoalVelocity(PositionVector pose, PositionVector goal, uint8_t FId, FormationName Fna)
{
    VelocityVector  GoalVelocity(2, 0);
    PositionVector  Bias(2, 0);
    double          BiasMag = 0.0;

    Bias[X] = goal[X] - pose[X] + FormationBias_01[Fna][FId][X];
    Bias[Y] = goal[Y] - pose[Y] + FormationBias_01[Fna][FId][Y];
    BiasMag = SquaredNorm(Bias);
    
    if (BiasMag < DEADZONE) {
        return GoalVelocity;
    }

    GoalVelocity[X] = Bias[X];
    GoalVelocity[Y] = Bias[Y];
    NormaliseVelocity(GoalVelocity);
    if (BiasMag < CONTROLZONE) {
        GoalVelocity[X] = GoalVelocity[X] * sin(M_PI_2 * BiasMag / CONTROLZONE);
        GoalVelocity[Y] = GoalVelocity[Y] * sin(M_PI_2 * BiasMag / CONTROLZONE);
    }

    return GoalVelocity;
}

