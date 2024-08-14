#include "../include/formation.h"
#include "../include/goal.h"

#include <numeric>

static PositionVector GetDesiredPosition(PositionVector LPos, uint8_t FId, 
    FormationName Fna, FormationMode Fmode)
{
    PositionVector BiasPos = {0.0, 0.0};
    PositionVector DesiredPos = {0.0, 0.0};
    double theta = LPos[YAW] - M_PI_2;

    if (Fmode == LEADER) {
        if (ALLOW_FORMATION_ROTATION) {
            BiasPos[X] = FormationBias[Fna][FId][X] * cos(theta) - FormationBias[Fna][FId][Y] * sin(theta); 
            BiasPos[Y] = FormationBias[Fna][FId][X] * sin(theta) + FormationBias[Fna][FId][Y] * cos(theta);
        } else {
            BiasPos[X] = FormationBias[Fna][FId][X];
            BiasPos[Y] = FormationBias[Fna][FId][Y];
        }
    } else if (Fmode == CENTER) {
        BiasPos[X] = FormationBias_01[Fna][FId][X];
        BiasPos[Y] = FormationBias_01[Fna][FId][Y];
    }

    DesiredPos[X] = LPos[X] + BiasPos[X];
    DesiredPos[Y] = LPos[Y] + BiasPos[Y];

    return DesiredPos;
}

VelocityVector GetFormationVelocity(PositionVector LPos, PositionVector FPos, 
    uint8_t FId, FormationName Fna, FormationMode Fmode, double* Err)
{
    PositionVector  GoalPos;
    VelocityVector  FormationVelocity(2, 0);
    PositionVector  Bias(2, 0);
    double          BiasMag = 0.0;

    GoalPos = GetDesiredPosition(LPos, FId, Fna, Fmode);
    Bias[X] = GoalPos[X] - FPos[X];
    Bias[Y] = GoalPos[Y] - FPos[Y];
    BiasMag = SquaredNorm(Bias);
    if (Err != nullptr) {
        *Err = BiasMag;
    }
    
    if (BiasMag < DEADZONE) {
        return FormationVelocity;
    }

    FormationVelocity[X] = Bias[X] ;
    FormationVelocity[Y] = Bias[Y] ;
    NormaliseVelocity(FormationVelocity);

    if (BiasMag < CONTROLZONE) {
        FormationVelocity[X] = FormationVelocity[X] * sin(M_PI_2 * BiasMag / CONTROLZONE);
        FormationVelocity[Y] = FormationVelocity[Y] * sin(M_PI_2 * BiasMag / CONTROLZONE);
    }
    
    return FormationVelocity;
}

VelocityVector GetFormationVelocity(vector<PositionVector>& RobotPos,
    vector<vector<int>>& Topo, uint8_t FId, FormationName Fna, double* Err)
{
    int topo_count = 0;
    double BiasMag = 0.0;
    int robot_num = RobotPos.size();
    VelocityVector  FormationVelocity(2, 0);
    PositionVector  Bias(2, 0);
    PositionVector  CurPos = RobotPos[FId];

    if (Err != nullptr) {
        *Err = 0;
    }
    
    for (int j = 0; j < robot_num; ++j) {
        if (j == FId) {
            continue;
        }
        Bias[X] = RobotPos[j][X] + FormationMatrix[Fna][FId][j][X] - CurPos[X];
        Bias[Y] = RobotPos[j][Y] + FormationMatrix[Fna][FId][j][Y] - CurPos[Y];
        BiasMag = SquaredNorm(Bias);
        if (Err != nullptr) {
            *Err += BiasMag;
        }

        if (BiasMag < DEADZONE || !Topo[FId][j]) {
            continue;
        }
        if (BiasMag < CONTROLZONE) {
            Bias[X] = Bias[X] * sin(M_PI_2 * BiasMag / CONTROLZONE);
            Bias[Y] = Bias[Y] * sin(M_PI_2 * BiasMag / CONTROLZONE);
        }
        FormationVelocity[X] += Bias[X] ;
        FormationVelocity[Y] += Bias[Y] ;
    }
    NormaliseVelocity(FormationVelocity);
    if (Err != nullptr) {
        topo_count = std::accumulate(Topo[FId].begin(), Topo[FId].end(), 1);
        *Err /= topo_count;
        if (*Err < 3 * CONTROLZONE) {
            FormationVelocity[X] = FormationVelocity[X] * sin(M_PI_2 * BiasMag / (3*CONTROLZONE));
            FormationVelocity[Y] = FormationVelocity[Y] * sin(M_PI_2 * BiasMag / (3*CONTROLZONE));
        }
    }

    return FormationVelocity;
}

