#include "urdf_helper.hpp"

urdf::ModelInterfaceSharedPtr load_urdf(std::string file_path,std::string resources_path)
{
    //URDFの読み込み
    std::string xml_string;
    std::fstream xml_file(file_path, std::fstream::in);
    if (xml_file.is_open())
    {
        while (xml_file.good())
        {
            std::string line;
            std::getline(xml_file, line);

            //package://[パッケージ名]を、file:///に変換する
            std::regex re(R"((.+)package:\/\/.*?\/(.*\..+))");
            std::string fmt = "$1file://"+resources_path+"/$2";
            line = std::regex_replace(line, re, fmt);

            xml_string += (line + "\n");
        }


        xml_file.close();

        auto robot_model = urdf::parseURDF(xml_string);
        return robot_model;
    }

    return nullptr;
}

struct jointName{
    using type = std::vector<std::string>;
    jointName(type& _result):result(_result){}

    void operator ()(urdf::JointSharedPtr joint){
        if(joint->mimic == nullptr){
            result.push_back(joint->name);
        }else{
            LOG_DEBUG_STREAM()<<"mimic joint: "<<joint->name<<LOG_END;
        }
    }

    type &result;
};

struct addMimicValue{
    using type = std::map<std::string, double>;
    addMimicValue(type& _result):result(_result){}

    void operator ()(urdf::JointSharedPtr joint){
        if(joint->mimic == nullptr){
            return;
        }

        auto mult = joint->mimic->multiplier;
        auto ofst = joint->mimic->offset;
        auto jval = result[joint->mimic->joint_name] * mult + ofst;

        result.insert(std::make_pair(joint->name, jval));
    }

    type &result;
};

struct jointLimit{
    using type = std::map<std::string,std::pair<double,double>>;
    jointLimit(type& _result):result(_result){}

    void operator ()(urdf::JointSharedPtr joint){
        std::string jname = joint->name;
        double upper = 0;
        double lower = 0;
        if(joint->limits){
            upper = joint->limits->upper;
            lower = joint->limits->lower;
        }
        result[jname] = std::make_pair(lower, upper);
    }

    type &result;
};

struct JointAllLimit{
    using type = std::map<std::string,JointLimitType>;
    JointAllLimit(type& _result):result(_result){}

    void operator ()(urdf::JointSharedPtr joint){
        std::string jname = joint->name;
        if(joint->limits){
            JointLimitType limit;
            limit.upper = joint->limits->upper;
            limit.lower = joint->limits->lower;
            limit.velocity = joint->limits->velocity;
            result[jname] = limit;
        }
    }

    type &result;
};


template<class T>
static bool traverse_joints(urdf::LinkConstSharedPtr link, T &result)
{
    for (const urdf::LinkSharedPtr & child : link->child_links)
    {
        if (child && child->parent_joint) {
            if(child->parent_joint->type != urdf::Joint::UNKNOWN && child->parent_joint->type != urdf::Joint::FIXED)
            {
                result(child->parent_joint);
            }
            traverse_joints(child, result);
        } else {
            LOG_ERROR_STREAM()<<"root link: "<<link->name<<" has a null child!"<<LOG_END;
            return false;
        }
    }

    return true;
}

bool extractJointNames(urdf::ModelInterface &robot_model,std::vector<std::string> &joint_names)
{
    jointName jname(joint_names);
    return traverse_joints(robot_model.getRoot(),jname);
}

bool extractJointLimits(urdf::ModelInterface &robot_model,std::map<std::string,std::pair<double,double>>& joint_limits)
{
    jointLimit jlimits(joint_limits);
    return traverse_joints(robot_model.getRoot(),jlimits);
}


bool extractJointLimits(urdf::ModelInterface &robot_model,std::map<std::string,JointLimitType> & joint_limits)
{
    JointAllLimit jlimits(joint_limits);
    return traverse_joints(robot_model.getRoot(),jlimits);
}


bool addMimicJointAngles(urdf::ModelInterface &robot_model,std::map<std::string, double> &joint_state_map)
{
    addMimicValue jvalue(joint_state_map);
    return traverse_joints(robot_model.getRoot(),jvalue);
}
