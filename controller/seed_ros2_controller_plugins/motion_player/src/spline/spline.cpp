#include <Eigen/Dense>

#include "spline.hpp"

namespace motion_player {

bool get_spline_coeff(double end_sec, const JointState2 &prev_tgt, const JointState2 &next_tgt,double &a0,double &a1,double &a2,double &a3,double &a4,double &a5){

    double t1 = std::pow(end_sec, 1);
    double t2 = std::pow(end_sec, 2);
    double t3 = std::pow(end_sec, 3);
    double t4 = std::pow(end_sec, 4);
    double t5 = std::pow(end_sec, 5);

    double pos0 = prev_tgt.pos;
    double vel0 = prev_tgt.vel;
    double acc0 = prev_tgt.acc;

    double pos1 = next_tgt.pos;
    double vel1 = next_tgt.vel;
    double acc1 = next_tgt.acc;


    Eigen::Matrix3d mat;
    mat<<   t3,    t4,    t5,
          3*t2,  4*t3,  5*t4,
          6*t1, 12*t2, 20*t3;

    Eigen::Vector3d vq;
    vq << pos1 - pos0 - vel0 * t1 - 0.5 * acc0 * t2,
          vel1 - vel0 - acc0 * t1,
          acc1 - acc0;

    Eigen::Vector3d va = mat.inverse() * vq;

    a0 = pos0;
    a1 = vel0;
    a2 = 0.5 * acc0;
    a3 = va(0);
    a4 = va(1);
    a5 = va(2);
    return true;
}

bool interpolate(double cur_sec, double end_sec, const JointState2 &prev_tgt, const JointState2 &next_tgt, double& pos,double &vel,double&acc,double&jerk) {
    double a0,a1,a2,a3,a4,a5;
    get_spline_coeff(end_sec,prev_tgt,next_tgt,a0,a1,a2,a3,a4,a5);

    double jpos =
              a0
            + a1      * cur_sec
            + a2      * std::pow(cur_sec, 2)
            + a3      * std::pow(cur_sec, 3)
            + a4      * std::pow(cur_sec, 4)
            + a5      * std::pow(cur_sec, 5);

    double jvel =
              a1
            + a2      * 2 * cur_sec
            + a3      * 3 * std::pow(cur_sec, 2)
            + a4      * 4 * std::pow(cur_sec, 3)
            + a5      * 5 * std::pow(cur_sec, 4);

    double jacc =
              a2      * 2
            + a3      * 6  *  cur_sec
            + a4      * 12 * std::pow(cur_sec, 2)
            + a5      * 20 * std::pow(cur_sec, 3);

    double jjerk =
              a3      * 6
            + a4      * 24 * cur_sec
            + a5      * 60 * std::pow(cur_sec, 2);

    pos = jpos;
    vel = jvel;
    acc = jacc;
    jerk = jjerk;

    return true;
}


bool interpolate(int step, double dt, int step_st, int step_end, const JointState2 &prev_tgt, const JointState2 &next_tgt, JointState2 &cur_tgt) {
    double jerk = 0;
    double diff_sec = (step_end - step_st) * dt;
    double cur_sec = (step - step_st) * dt;
    return interpolate(cur_sec, diff_sec, prev_tgt, next_tgt, cur_tgt.pos, cur_tgt.vel, cur_tgt.acc, jerk);
}


}
