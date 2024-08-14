#include <map>
#include <matplot/matplot.h>
#include <std_msgs/String.h>

#include "../include/cmdline.h"
#include "../include/b2_util.h"
#include "../include/tic_toc.hpp"
#include "../include/move_robot.hpp"
#include "../include/formation_controller.h"

using namespace matplot;

int ROBOT_NUM = 5;
std::map<string, FormationName> Str2Formation = {{"line", LINE}, 
    {"column",COLUMN}, {"diamond", DIAMOND}, {"wedge", WEDGE}};
std::map<string, FormationMode> Str2Mode = {{"leader", LEADER}, 
    {"center",CENTER}, {"neighbor", NEIGHBOR}};
vector<MoveRobot*> Robots;
vector<vector<vector<double>>> Trajectory(ROBOT_NUM, vector<vector<double>>(2));
vector<vector<double>> AbsoluteError(ROBOT_NUM);
vector<vector<int>> Topology;
double Err = 0;
int ArriveCount = 0;
PositionVector GoalPositon = {0, 3.0};
FormationName ROBOT_FORMATION = LINE;
FormationMode ROBOT_MODE = LEADER;
PositionVector CenterPos = {0, 0};

void KeyboardCallback(const std_msgs::String::ConstPtr& msg);

int main(int argc, char* argv[])
{
    ros::init(argc, argv, "formation_move");
    ros::NodeHandle n;
    ros::Rate loop_rate(CONTROLRATE);
    ros::Subscriber sub = n.subscribe("keyboard", 1000, KeyboardCallback);

    cmdline::parser cmd;
    cmd.add<int>("robot_num", 'n', "The number of robots", false, 5, cmdline::range(1, 10));
    cmd.add<double>("goal_x", 'x', "The goal_x of the leader robot", false, GoalPositon[0]);
    cmd.add<double>("goal_y", 'y', "The goal_y of the leader robot", false, GoalPositon[1]);
    cmd.add<string>("formation", 'f', "The type of formation", false, "line",
        cmdline::oneof<string>("line", "column", "diamond", "wedge"));
    cmd.add<string>("mode", 'm', "The mode of formation", false, "leader",
        cmdline::oneof<string>("leader", "center", "neighbor"));
    cmd.parse_check(argc, argv);
    ROBOT_NUM = cmd.get<int>("robot_num");
    GoalPositon[X] = cmd.get<double>("goal_x");
    GoalPositon[Y] = cmd.get<double>("goal_y");
    ROBOT_FORMATION = Str2Formation[cmd.get<string>("formation")];
    ROBOT_MODE = Str2Mode[cmd.get<string>("mode")];
    Robots.resize(ROBOT_NUM);
    Trajectory.resize(ROBOT_NUM);
    AbsoluteError.resize(ROBOT_NUM);

    SpawnTargetModel(&n, GoalPositon);
    for (int i = 0; i < ROBOT_NUM; ++i) {
        Robots[i] = new MoveRobot(&n);
    }
    if (ROBOT_MODE == NEIGHBOR) {
        Topology = GetTopology("./src/PPIP/b2_formation_control/config/topology.ini");
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

    TicToc t_r;
    while (ros::ok()) {
        if (ROBOT_MODE == LEADER) {
            Robots[0]->SetVelocity(GetControllerOutput(Robots[0]->GetPose(), 
                GoalPositon, Robots[0]->GetLaser()));
            for (int i = 1; i < ROBOT_NUM; ++i) {
                Robots[i]->SetVelocity(GetControllerOutput(Robots[0]->GetPose(),
                    Robots[i]->GetPose(), i, ROBOT_FORMATION, Robots[i]->GetLaser() ,&Err));
                AbsoluteError[i].emplace_back(Err);
            }
        } else if (ROBOT_MODE == CENTER) {
            /* this stupid code, please forgive it! */
            vector<PositionVector> RobotPos(ROBOT_NUM);
            for (int i = 0; i < ROBOT_NUM; ++i) {
                RobotPos[i] = Robots[i]->GetPose();
            }
            CenterPos = GetUnitCenter(MoveRobot::AllRobotPos);
            for (int i = 0; i < ROBOT_NUM; ++i) {
                Robots[i]->SetVelocity(GetControllerOutput(CenterPos, RobotPos[i], 
                    GoalPositon, i, ROBOT_FORMATION, Robots[i]->GetLaser() ,&Err));
                AbsoluteError[i].emplace_back(Err);
            }
        } else if (ROBOT_MODE == NEIGHBOR) {
            vector<PositionVector> RobotPos(ROBOT_NUM);
            for (int i = 0; i < ROBOT_NUM; ++i) {
                RobotPos[i] = Robots[i]->GetPose();
            }
            CenterPos = GetUnitCenter(MoveRobot::AllRobotPos);
            for (int i = 0; i < ROBOT_NUM; ++i) {
                Robots[i]->SetVelocity(GetControllerOutput(RobotPos, Topology, i,
                    GoalPositon, ROBOT_FORMATION, Robots[i]->GetLaser() ,&Err));
                AbsoluteError[i].emplace_back(Err);
            }
        }

        for (int i = 0; i < ROBOT_NUM; ++i) {
            PositionVector pos = Robots[i]->GetPose();
            Trajectory[i][X].emplace_back(pos[X]);
            Trajectory[i][Y].emplace_back(pos[Y]);
        }

        if (IsArriveGoal(Robots[0]->GetPose(), GoalPositon) ||
            IsArriveGoal(CenterPos, GoalPositon)) {
            if (++ArriveCount > (3 * CONTROLRATE)) {
                break;
            }
        }

        ros::spinOnce();
        loop_rate.sleep();
    }
    ROS_INFO("Whole time costs: %.2f s.", t_r.toc() / 1000 - 3);
    
    
    vector<double> x = {GoalPositon[0]};
    vector<double> y = {GoalPositon[1]};
    auto l = scatter(x, y, 23);
    l->marker_face(true);
    l->marker_style(line_spec::marker_style::upward_pointing_triangle);
    hold(on);
    plot(Trajectory[0][X],Trajectory[0][Y])->line_width(2);
    for (int i = 1; i < ROBOT_NUM; ++i) {
        plot(Trajectory[i][X],Trajectory[i][Y])->line_width(2);
    }
    hold(off);
    grid(true);
    legend("goal" ,"robot0", "robot1", "robot2", "robot3", "robot4");
    xlabel("X");
    ylabel("Y");
    title("The Trajectory of Robots");
    show();

    plot(AbsoluteError[1])->line_width(2);
    hold(on);
    for (int i = 2; i < ROBOT_NUM; ++i) {
        plot(AbsoluteError[i])->line_width(2);
    }
    if (ROBOT_MODE == CENTER || ROBOT_MODE == NEIGHBOR) {
        plot(AbsoluteError[0])->line_width(2);
    }
    hold(off);
    if (ROBOT_MODE == LEADER) {
        legend("robot1", "robot2", "robot3", "robot4");
    } else if (ROBOT_MODE == CENTER || ROBOT_MODE == NEIGHBOR) {
        legend("robot0", "robot1", "robot2", "robot3", "robot4");
    }
    xlabel("Time Step");
    ylabel("Absolute Error");
    title("The AE of Formation");
    show();


    for (int i = 0; i < ROBOT_NUM; ++i) {
        delete Robots[i];
    }
    return 0;
}

void KeyboardCallback(const std_msgs::String::ConstPtr& msg)
{
    string msgstr = msg->data;
 
    if (msgstr == "line" || msgstr == "l") {
        ROBOT_FORMATION = LINE;
        ROS_INFO("Formation switch to %s.", "line");
    } else if (msgstr == "column" || msgstr == "c") {
        ROBOT_FORMATION = COLUMN;
        ROS_INFO("Formation switch to %s.", "column");
    } else if (msgstr == "diamond" || msgstr == "d") {
        ROBOT_FORMATION = DIAMOND;
        ROS_INFO("Formation switch to %s.", "diamond");
    } else if (msgstr == "wedge" || msgstr == "w") {
        ROBOT_FORMATION = WEDGE;
        ROS_INFO("Formation switch to %s.", "wedge");
    }
}

