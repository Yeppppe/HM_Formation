#pragma once
#ifndef __MOVE_ROBOT_HPP
#define __MOVE_ROBOT_HPP

#include <ros/ros.h>
#include <geometry_msgs/Twist.h>
#include <sensor_msgs/LaserScan.h>
#include <tf/transform_datatypes.h>
#include <gazebo_msgs/ModelStates.h>

#include "common_include.h"

class GroundtruthPose {
public:
    inline GroundtruthPose(string rName, ros::NodeHandle* n);
    ~GroundtruthPose(void) {}
    inline void DataCallback(const gazebo_msgs::ModelStates::ConstPtr& msg);
    inline bool Ready(void) const;
    inline string GetRobotName(void) const;
    inline PositionVector GetRobotPose(void) const;
    
private:
    ros::Subscriber GroundSub;
    string RobotName;
    PositionVector RobotPose;
};

class SimpleLaser {
public:
    inline SimpleLaser(string rName, ros::NodeHandle* n);
    ~SimpleLaser(void) { }
    inline void DataCallback(const sensor_msgs::LaserScan::ConstPtr& msg);
    inline bool Ready(void) const;
    inline string GetRobotName(void) const;
    inline LaserVector GetMeasurement(void) const;

private:
    static const vector<double> SampleAngle;

    ros::Subscriber ScanSub;
    string RobotName;
    double SampleWidth = M_PI / 180. * 10;
    LaserVector Measurement;
    vector<vector<int>> SampleIndices;

    inline bool within(double angle, double left, double right);
};

class MoveRobot {
public:
    MoveRobot(ros::NodeHandle* n);
    ~MoveRobot(void);
    PositionVector GetPose(void) const;
    LaserVector GetLaser(void) const;
    void SetVelocity(LinearAngularVector velocity);
    bool DetectCorridor(void);
    bool GetReady(void);
    static vector<PositionVector> AllRobotPos;

private:
    static int CurRobotIndex;
    int RobotIndex;
    string RobotName;
    ros::Publisher   CmdVelPub;
    GroundtruthPose* GroundTruth;
    SimpleLaser*     Laser;
    LinearAngularVector RobotVelocity;
};

#endif

