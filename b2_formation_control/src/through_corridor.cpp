#include "../include/b2_util.h"
#include "../include/move_robot.hpp"
#include "../include/formation_controller.h"

using namespace std;

int ROBOT_NUM = 5;
vector<MoveRobot*> Robots;
FormationName ROBOT_FORMATION = DIAMOND;
vector<PositionVector> Waypoints = {{-1, -3}, {-1, 0.5}, {0.2, 1.2}, {0.2, 5.6}};
int way_index = 0;

int main(int argc, char* argv[])
{
    ros::init(argc, argv, "through_corridor");
    ros::NodeHandle n;
    ros::Rate loop_rate(200);

    SpawnTargetModel(&n, Waypoints.back());
    Robots.resize(ROBOT_NUM);
    for (int i = 0; i < ROBOT_NUM; ++i) {
        Robots[i] = new MoveRobot(&n);
    } 

    while (ros::ok()) {
        if (!AllRobotsReady(Robots)) {
            ROS_INFO("Sensors are not ready!");
            sleep(1);
            ros::spinOnce();
        } else {
            ROS_INFO("Sensors are ready!");
            break;
        }
    }

    while (ros::ok()) {
        Robots[0]->SetVelocity(GetControllerOutput(Robots[0]->GetPose(), 
            Waypoints[way_index], Robots[0]->GetLaser()));
        for (uint8_t i = 1; i < ROBOT_NUM; ++i) {
            Robots[i]->SetVelocity(GetControllerOutput(Robots[0]->GetPose(),
                Robots[i]->GetPose(), i, ROBOT_FORMATION, Robots[i]->GetLaser()));
        }
        
        if (IsArriveGoal(Robots[0]->GetPose(),Waypoints.back())) {
            break;
        }
        if (IsArriveGoal(Robots[0]->GetPose(),Waypoints[way_index])) {
            way_index++;
        }
    
        if (IsCorridor(Robots) && ROBOT_FORMATION != COLUMN) {
             ROS_INFO("Detect corridor, switch formation to LINE.");
            ROBOT_FORMATION = COLUMN;
        } else if (!IsCorridor(Robots) && ROBOT_FORMATION != DIAMOND) {
            ROS_INFO("No corridor, switch formation to DIAMOND.");
            ROBOT_FORMATION = DIAMOND;
        }

        ros::spinOnce();
        loop_rate.sleep();
    }
    
    for (int i = 0; i < ROBOT_NUM; ++i) {
        delete Robots[i];
    }
    return 0;
}

