#include <fstream>
#include <gazebo_msgs/SpawnModel.h>

#include "../include/b2_util.h"
#include "../include/goal.h"

bool AllRobotsReady(const vector<MoveRobot*>& robots) 
{
    for (MoveRobot* r : robots) {
        try {
            if (!r->GetReady()) {
                return false;
            }
        } catch (...) {
            ROS_ERROR("Try Robot->GetReady ERROR, did the robot instance really exist?");
        }
    }
    return true;
}

bool IsCorridor(const vector<MoveRobot*>& robots) 
{
    static int yes_count = 0;
    static int not_count = 0;
    static bool last_return = false;
    int corridor_count = 0;

    for (MoveRobot* r : robots) {
        try {
            if (r->DetectCorridor()) {
                corridor_count++;
            }
        } catch (...) {
            ROS_ERROR("Try Robot->DetectCorridor ERROR, did the robot instance really exist?");
        }
    }
    if (robots[0]->DetectCorridor() || corridor_count >= (robots.size() / 2)) {
        if (yes_count < 200) {
            yes_count++;
        }
        not_count = 0;
    } else {
        if (not_count < 200) {
            not_count++;
        }
        yes_count = 0;
    }

    if (yes_count >= 20) {
        last_return = true;   
    } else if (not_count >= 20) {
        last_return = false;
    }

    return last_return;
}

bool SpawnTargetModel(ros::NodeHandle* n, PositionVector tar_pos)
{
    static int target_model_count = 0;
    ros::ServiceClient client = n->serviceClient<gazebo_msgs::SpawnModel>("/gazebo/spawn_sdf_model");

    string sdf_file_path = "./src/PPIP/scene_loading/urdf/target.sdf";
    std::ifstream infile(sdf_file_path);
    if(!infile.is_open()) {
        ROS_ERROR("Open target sdf file failed!");
        return false;
    }
    std::ostringstream tmp;
    tmp << infile.rdbuf();
    string sdf_file = tmp.str();

    geometry_msgs::Pose initial_pose;
    initial_pose.position.x = tar_pos[X];
    initial_pose.position.y = tar_pos[Y];
    
    gazebo_msgs::SpawnModel sw;
    sw.request.model_name = "target_" + std::to_string(target_model_count);
    sw.request.model_xml = sdf_file;
    sw.request.initial_pose = initial_pose;
    sw.request.reference_frame = "world";
    client.waitForExistence();
    if (client.call(sw)) {
        // ROS_INFO(sw.response.status_message.c_str());
    } else {
        ROS_ERROR("Failed to call service add_two_ints");
    }
    target_model_count++;
    
    return true;
}

bool IsArriveGoal(PositionVector cur, PositionVector goal)
{
    PositionVector  Bias(2, 0);
    double          BiasMag = 0.0;
    Bias[X] = cur[X] - goal[X];
    Bias[Y] = cur[Y] - goal[Y];
    BiasMag = SquaredNorm(Bias);

    return BiasMag <= (3 * DEADZONE);
}

PositionVector GetUnitCenter(const vector<PositionVector>& robot_pos)
{
    int robot_num = robot_pos.size();
    double total_posx = 0;
    double total_posy = 0;
    PositionVector res = {0, 0};

    for (int i = 0; i < robot_num; ++i) {
        total_posx += robot_pos[i][X];
        total_posy += robot_pos[i][Y];
    }
    res[X] = total_posx / robot_num;
    res[Y] = total_posy / robot_num;
    
    return res;
}

vector<vector<int>> GetTopology(string config_path)
{
    vector<vector<int>> topo;
    std::ifstream inFile(config_path, std::ios::in);
    
    if (!inFile) {
        std::cerr << "Open " << config_path << " failed !\n";
        exit(1);
    }
    int num;
    int tmp; 
    string line;
    inFile >> line;
    num = atoi(line.substr(10).c_str());
    topo.resize(num);
    for (int i = 0; i < num; ++i) {
        for (int j = 0; j < num; ++j) {
            inFile >> tmp;
            topo[i].emplace_back(tmp);
        }
    }

    return topo;
}

