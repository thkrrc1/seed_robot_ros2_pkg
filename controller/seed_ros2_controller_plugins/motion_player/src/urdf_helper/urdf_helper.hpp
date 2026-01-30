#pragma once

#include <urdf_parser/urdf_parser.h>
#include "common.hpp"

struct JointLimitType{
    double upper;
    double lower;
    double velocity;
};

urdf::ModelInterfaceSharedPtr load_urdf(std::string file_path,std::string resources_path);
bool extractJointNames(urdf::ModelInterface &robot_model,std::vector<std::string> &joint_names);
bool extractJointLimits(urdf::ModelInterface &robot_model,std::map<std::string,std::pair<double,double>>& joint_limits);
bool extractJointLimits(urdf::ModelInterface &robot_model,std::map<std::string,JointLimitType> & joint_limits);
bool addMimicJointAngles(urdf::ModelInterface &robot_model,std::map<std::string, double> &joint_state_map);
