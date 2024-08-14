#include "../include/move_robot.hpp"


GroundtruthPose::GroundtruthPose(string rName, ros::NodeHandle* n)
{
    this->RobotName = rName;
    this->RobotPose.resize(3);
    this->RobotPose[X] = INFINITY;
    this->RobotPose[Y] = INFINITY;
    this->RobotPose[YAW] = INFINITY;
    this->GroundSub = n->subscribe("/gazebo/model_states", 1, &GroundtruthPose::DataCallback, this);
}

void GroundtruthPose::DataCallback(const gazebo_msgs::ModelStates::ConstPtr& msg)
{
    uint8_t ModelStatesIndex = -1; 
    for (int i = 0; i < msg->name.size(); ++i) {
        if (msg->name[i] == this->RobotName) {
            ModelStatesIndex = i;
            break;
        }
    }
    if (ModelStatesIndex == -1) {
        return ;
    }
    
    this->RobotPose[X] = msg->pose[ModelStatesIndex].position.x;
    this->RobotPose[Y] = msg->pose[ModelStatesIndex].position.y;
    
    tf::Quaternion quat;
    double roll,pitch, yaw;
    tf::quaternionMsgToTF(msg->pose[ModelStatesIndex].orientation, quat);
    tf::Matrix3x3(quat).getRPY(roll, pitch, yaw);
    this->RobotPose[YAW] = yaw;
}

bool GroundtruthPose::Ready(void) const
{
    return !(this->RobotPose[YAW] == INFINITY);
}

string GroundtruthPose::GetRobotName(void) const
{
    return this->RobotName;
}

PositionVector GroundtruthPose::GetRobotPose(void) const
{
    return this->RobotPose;
}


const vector<double> SimpleLaser::SampleAngle = {0., M_PI_4, M_PI_4 * 7, M_PI_2, M_PI_2 * 3};

SimpleLaser::SimpleLaser(string rName, ros::NodeHandle* n)
{
    // ROS_INFO("SimpleLaser contruct function ...");

    this->RobotName = rName;
    this->Measurement.resize(SimpleLaser::SampleAngle.size());
    for (auto& m : this->Measurement) {
        m = -1;
    }
    this->ScanSub = n->subscribe("/"+rName+"/scan", 1000, &SimpleLaser::DataCallback, this);
}

void SimpleLaser::DataCallback(const sensor_msgs::LaserScan::ConstPtr& msg)
{
    // ROS_INFO("SimpleLaser DataCallback ...");
    if (this->SampleIndices.empty()) {
        this->SampleIndices.resize(SimpleLaser::SampleAngle.size());
        for (int i = 0; i < msg->ranges.size(); ++i) {
            double angle = msg->angle_min + i * msg->angle_increment;
            for (int j = 0; j < SimpleLaser::SampleAngle.size(); ++j) {
                double CenterAngle = SimpleLaser::SampleAngle[j];
                if (within(angle, CenterAngle - this->SampleWidth / 2, 
                    CenterAngle + this->SampleWidth / 2)) {
                    this->SampleIndices[j].emplace_back(i);
                }
            }
        }
    }

    for (int i = 0; i < this->SampleIndices.size(); ++i) {
        double MinDistance = -1.0;
        int MinIndex = -1;
        for (int _index : this->SampleIndices[i]) {
            if (MinIndex == -1.0) {
                MinDistance = msg->ranges[_index];
                MinIndex = _index;
            } else if (MinDistance > msg->ranges[_index]){
                MinDistance = msg->ranges[_index];
                MinIndex = _index;
            }
        }
        this->Measurement[i] = MinDistance;
        if (i == 0 && MinDistance != -1 && MinIndex < 30) {
            this->Measurement[i] = -MinDistance; 
        } 
    }
}

bool SimpleLaser::Ready(void) const
{
    return !(this->Measurement[0] == -1);
}

string SimpleLaser::GetRobotName(void) const
{
    return this->RobotName;
}

LaserVector SimpleLaser::GetMeasurement(void) const
{
    return this->Measurement;
}

bool SimpleLaser::within(double angle, double left, double right)
{
    double PI2 = M_PI * 2;
    if (left < 0) {
        left += PI2;
    } else if (left > M_PI_2 && left < M_PI_2 * 3) {
        left = M_PI_2 * 3;
    }
    if (right < 0) {
        right += PI2;
    } else if (right > M_PI_2 && right < M_PI_2 * 3){
        right = M_PI_2;
    }
    if (left < right) {
        return left <= angle && angle <= right;
    } 
    return left <= angle || angle <= right;
}


int MoveRobot::CurRobotIndex = 0;
vector<PositionVector> MoveRobot::AllRobotPos = {};

MoveRobot::MoveRobot(ros::NodeHandle* n)
{
    this->RobotIndex = CurRobotIndex;
    this->RobotName = "tb3_" + std::to_string(this->RobotIndex);
    this->CmdVelPub = n->advertise<geometry_msgs::Twist>("/" + this->RobotName + "/cmd_vel", 1000);
    this->GroundTruth = new GroundtruthPose(this->RobotName, n);
    this->Laser = new SimpleLaser(this->RobotName, n);
    CurRobotIndex++;
    AllRobotPos.resize(CurRobotIndex);
}

MoveRobot::~MoveRobot(void)
{
    delete this->GroundTruth;
    delete this->Laser;
}

PositionVector MoveRobot::GetPose(void) const
{
    AllRobotPos[this->RobotIndex] = this->GroundTruth->GetRobotPose();
    return AllRobotPos[this->RobotIndex];
}

LaserVector MoveRobot::GetLaser(void) const
{
    return this->Laser->GetMeasurement();
}

void MoveRobot::SetVelocity(LinearAngularVector velocity)
{
    geometry_msgs::Twist VelMsgs;
    VelMsgs.linear.x = velocity[UV];
    VelMsgs.angular.z = velocity[UW];

    this->CmdVelPub.publish(VelMsgs);
}

bool MoveRobot::DetectCorridor(void)
{
    LaserVector LVec = this->Laser->GetMeasurement();
    if (LVec[RIGHT] < 0.8 && LVec[LEFT] < 0.8 && 
        LVec[RIGHTFRONT] < 2 && LVec[LEFTFRONT] < 2) {
        return true;
    }
    return false;
}

bool MoveRobot::GetReady(void)
{
    if (this->Laser->Ready() && this->GroundTruth->Ready()) {
        return true;
    }
    return false;
}

