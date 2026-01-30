#pragma once

#include "joint_state.hpp"

namespace motion_player {
bool get_spline_coeff(double end_sec, const JointState2 &prev_tgt, const JointState2 &next_tgt,double &a0,double &a1,double &a2,double &a3,double &a4,double &a5);
bool interpolate(double cur_sec, double end_sec, const JointState2 &prev_tgt, const JointState2 &next_tgt, double& pos,double &vel,double&acc,double&jerk);
bool interpolate(int step, double dt, int step_st, int step_end, const JointState2 &prev_tgt, const JointState2 &next_tgt, JointState2 &cur_tgt);
}
