#include <map>
#include "stroke_converter.h"

void seed_converter::LifterMover::makeTables() {
    lifter_csv_dir = ament_index_cpp::get_package_share_directory("lifter_mover") + "/csv/typeG_lifter";

    
    if (makeTable(leg.table, lifter_csv_dir + "/leg.csv"))
        makeInvTable(leg.inv_table, leg.table);
}

int16_t seed_converter::LifterMover::calcStroke1(double angle,double scale,double offset){
    static const double rad2Deg = 180.0 / M_PI;
    if (std::isnan(angle)) {
        return 0x7FFF;
    }

    return static_cast<int16_t>(scale * (rad2Deg * angle + offset));
}

int16_t seed_converter::LifterMover::calcStroke2(double angle,double scale,int dir,double offset, const std::vector<seed_converter::StrokeMap> &_table){
    static const double rad2Deg = 180.0 / M_PI;
    if (std::isnan(angle)) {
        return 0x7FFF;
    }

    return static_cast<int16_t>(scale * setAngleToStroke(offset + dir * rad2Deg * angle, _table));
}


void seed_converter::LifterMover::calcStroke3(double angle1,double angle2,double scale,int dir1,int dir2, const std::vector<seed_converter::StrokeMap> &_table1, const std::vector<seed_converter::StrokeMap> &_table2,bool is_pitch,int16_t& out1,int16_t& out2){
    static const double rad2Deg = 180.0 / M_PI;
    if (std::isnan(angle1) || std::isnan(angle2)) {
        out1 = 0x7FFF;
        out2 = 0x7FFF;
        return;
    }

    seed_converter::DiffJoint value = setDualAngleToStroke(dir2*rad2Deg * angle2, dir1*rad2Deg * angle1, _table2, _table1, is_pitch);
    out1 = static_cast<int16_t>(scale * value.one);
    out2 = static_cast<int16_t>(scale * value.two);

    return;
}


void seed_converter::LifterMover::Angle2Stroke(std::vector<int16_t> &_strokes, const std::vector<double> &_angles) {
    static const float scale = 100.0;

    for(size_t idx = 0;idx < _strokes.size();++idx){
        _strokes[idx] = 0x7FFF;
    }

    _strokes[idx_knee] = calcStroke2(_angles[idx_knee],scale,-1,0, leg.table);
    _strokes[idx_ankle] = calcStroke2(_angles[idx_ankle],scale,1,0, leg.table);    

    // std::cout << "A2S:" <<  "idx_knee:" << _strokes[idx_knee] << "," << "idx_ankle:" << _strokes[idx_ankle]  << std::endl;
    
    // std::cout << "A2S:" << "idx_waist_y:" << _strokes[idx_waist_y] << "," << "idx_waist_p:" << _strokes[idx_waist_p] << "," << "idx_waist_r:" << _strokes[idx_waist_r]  << ","
    //           << "idx_l_shoulder_p:"<< _strokes[idx_l_shoulder_p] << "," << "idx_l_shoulder_r:" << _strokes[idx_l_shoulder_r] << "," << "idx_l_shoulder_y:" << _strokes[idx_l_shoulder_y] << ","
    //           << "idx_l_elbow:"<< _strokes[idx_l_elbow] << "," << "idx_l_wrist_y:" << _strokes[idx_l_wrist_y] << "," << "idx_l_wrist_p:" << _strokes[idx_l_wrist_p] << ","
    //           << "idx_l_wrist_r:"<< _strokes[idx_l_wrist_r] << "," << "idx_l_thumb:" << _strokes[idx_l_thumb] << "," << "idx_neck_y:" << _strokes[idx_neck_y] << ","
    //           << "idx_neck_p:"<< _strokes[idx_neck_p] << "," << "," << "idx_neck_r:" << _strokes[idx_neck_r] << "," << "idx_r_shoulder_p:" << _strokes[idx_r_shoulder_p] << ","
    //           << "idx_r_shoulder_r:"<< _strokes[idx_r_shoulder_r] << "," << "idx_r_shoulder_y:" << _strokes[idx_r_shoulder_y] << "," << "idx_r_elbow:" << _strokes[idx_r_elbow] << ","
    //           << "idx_r_wrist_y:"<< _strokes[idx_r_wrist_y] << "," << "idx_r_wrist_p:" << _strokes[idx_r_wrist_p] << "," << "idx_r_wrist_r:" << _strokes[idx_r_wrist_r] << ","
    //           << "idx_r_thumb:"<< _strokes[idx_r_thumb] << "," << "idx_knee:" << _strokes[idx_knee] << "," << "idx_ankle:" << _strokes[idx_ankle] << std::endl;
}

//無限回転ホイールの回転角度を求める(-180 ~ 180度)
int16_t seed_converter::LifterMover::calcWheelAngDeg(int idx, int16_t raw_ang) {
    static constexpr int16_t max_ang = 13602; //最大値
    static constexpr int16_t min_ang = -13602; //最大値
    static constexpr int16_t max_ang_mod = max_ang % 360;
    static constexpr int16_t min_ang_mod = min_ang % 360;

    prev_wheel[idx] = cur_wheel[idx];
    cur_wheel[idx] = raw_ang;
    if (cur_wheel[idx] < -13000 && prev_wheel[idx] > 13000) {
        //順方向にオーバーフローした場合
        zero_ofst[idx] = (zero_ofst[idx] + max_ang_mod - min_ang_mod) % 360;
    } else if (prev_wheel[idx] < -13000 && cur_wheel[idx] > 13000) {
        //逆方向にオーバーフローした場合
        zero_ofst[idx] = (zero_ofst[idx] + min_ang_mod - max_ang_mod) % 360;
    }

    auto cur_ang = (cur_wheel[idx] + zero_ofst[idx])%360;
    if(cur_ang < 0){cur_ang += 360;}
    return (cur_ang - 180.);
}

void seed_converter::LifterMover::Stroke2Angle(std::vector<double> &_angles, const std::vector<int16_t> &_strokes) {
    static const float deg2Rad = M_PI / 180.0;
    static const float scale_inv = 0.01;    

    _angles[idx_knee] = -deg2Rad * setStrokeToAngle(scale_inv * _strokes[idx_knee], leg.inv_table);  // knee
    _angles[idx_ankle] = deg2Rad * setStrokeToAngle(scale_inv * _strokes[idx_ankle], leg.inv_table);  // ankle

    int16_t ang_fl = calcWheelAngDeg(0,_strokes[idx_wheel_front_left]);
    int16_t ang_fr = calcWheelAngDeg(1,_strokes[idx_wheel_front_right]);
    int16_t ang_rl = calcWheelAngDeg(2,_strokes[idx_wheel_rear_left]);
    int16_t ang_rr = calcWheelAngDeg(3,_strokes[idx_wheel_rear_right]);

    _angles[idx_wheel_front_left]  = deg2Rad * ang_fl;
    _angles[idx_wheel_front_right] = deg2Rad * ang_fr;
    _angles[idx_wheel_rear_left]   = deg2Rad * ang_rl;
    _angles[idx_wheel_rear_right]  = deg2Rad * ang_rr;

    // std::cout << "S2A:" << "idx_knee:" << _angles[idx_knee] << "," << "idx_ankle:" << _angles[idx_ankle]  << std::endl;

    // std::cout << "S2A:" << "idx_waist_y:" << _angles[idx_waist_y] << "," << "idx_waist_p:" << _angles[idx_waist_p] << "," << "idx_waist_r:" << _angles[idx_waist_r]  << ","
    //           << "idx_l_shoulder_p:"<< _angles[idx_l_shoulder_p]  << ","<< "idx_l_shoulder_r:" << _angles[idx_l_shoulder_r]  << ","<< "idx_l_shoulder_y:" << _angles[idx_l_shoulder_y] << ","
    //           << "idx_l_elbow:"<< _angles[idx_l_elbow]  << ","<< "idx_l_wrist_y:" << _angles[idx_l_wrist_y] << "," << "idx_l_wrist_p:" << _angles[idx_l_wrist_p] << ","
    //           << "idx_l_wrist_r:"<< _angles[idx_l_wrist_r]  << ","<< "idx_l_thumb:" << _angles[idx_l_thumb]  << "," << "idx_neck_y:" << _angles[idx_neck_y] << ","
    //           << "idx_neck_p:"<< _angles[idx_neck_p] << "," << "idx_neck_r:" << _angles[idx_neck_r] << "," << "idx_r_shoulder_p:" << _angles[idx_r_shoulder_p] << ","
    //           << "idx_r_shoulder_r:"<< _angles[idx_r_shoulder_r]  << ","<< "idx_r_shoulder_y:" << _angles[idx_r_shoulder_y] << "," << "idx_r_elbow:" << _angles[idx_r_elbow] << ","
    //           << "idx_r_wrist_y:"<< _angles[idx_r_wrist_y] << "," << "idx_r_wrist_p:" << _angles[idx_r_wrist_p] << "," << "idx_r_wrist_r:" << _angles[idx_r_wrist_r] << ","
    //           << "idx_r_thumb:"<< _angles[idx_r_thumb] << "," << "idx_knee:" << _angles[idx_knee] << "," << "idx_ankle:" << _angles[idx_ankle] << std::endl;
}

void seed_converter::LifterMover::setJointNames(const std::vector<std::string> &names) {
    std::map<std::string, int> jnameIndex;
    jnameIndex.clear();
    for (size_t idx = 0; idx < names.size(); ++idx) {
        jnameIndex.emplace(names[idx], idx);
    }
    
    idx_knee = jnameIndex["knee_joint"];
    idx_ankle = jnameIndex["ankle_joint"];
    idx_wheel_front_left = jnameIndex["wheel_front_left"];
    idx_wheel_front_right = jnameIndex["wheel_front_right"];
    idx_wheel_rear_left = jnameIndex["wheel_rear_left"];
    idx_wheel_rear_right = jnameIndex["wheel_rear_right"];
}

void seed_converter::LifterMover::calcActuatorVel(std::vector<int16_t> &actuator_vel, const std::vector<double> &joint_vel) {
    for (size_t idx = 0; idx < actuator_vel.size(); ++idx) {
        if (std::isnan(joint_vel[idx])) {
            actuator_vel[idx] = 0x7FFF;
        } else {
            actuator_vel[idx] = static_cast<int16_t>(joint_vel[idx] * 180.0 / M_PI);
        }
    }
}

void seed_converter::LifterMover::calcJointVel(std::vector<double> &joint_vel,const std::vector<int16_t> &actuator_vel){
    for (size_t idx = 0; idx < joint_vel.size(); ++idx) {
        if (actuator_vel[idx] == 0x7FFF) {
            joint_vel[idx] = std::numeric_limits<double>::quiet_NaN();
        } else {
            joint_vel[idx] = static_cast<double>(actuator_vel[idx] * M_PI / 180.0);
        }
    }
}

#include <pluginlib/class_list_macros.hpp>
PLUGINLIB_EXPORT_CLASS(seed_converter::LifterMover, seed_converter::StrokeConverter)
