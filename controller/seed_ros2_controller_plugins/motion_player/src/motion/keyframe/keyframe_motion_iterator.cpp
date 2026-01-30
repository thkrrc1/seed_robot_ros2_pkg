#include "keyframe_motion_iterator.hpp"
#include <Eigen/Dense>
#include <Eigen/LU>

namespace motion_editor2 {


KeyframeMotionIterator2::KeyframeMotionIterator2(const KeyframeMotion2 &motion):motion(motion) {
}

KeyframeMotionIterator2::~KeyframeMotionIterator2(){
}


double KeyframeMotionIterator2::getStepTime() {
    return motion.dt;
}

bool KeyframeMotionIterator2::hasElement(int step) {
    if (step >= 0 && step <= getMaxStep()) {
        this->step = step;
        return true;
    }
    return false;
}

int KeyframeMotionIterator2::getMaxStep() {
    int max_step = 0;
    if(motion.keyframe.size() != 0){
        int end_step = motion.keyframe.back().step;
        if(end_step > max_step){
            max_step = end_step;
        }
    }

    if(motion.audio.size() != 0){
        int end_step = motion.audio.back().step + motion.audio.back().dur_step;
        if(end_step > max_step){
            max_step = end_step;
        }
    }

    if(motion.video.size() != 0){
        int end_step = motion.video.back().step + motion.video.back().dur_step;
        if(end_step > max_step){
            max_step = end_step;
        }
    }

    return max_step;
}

bool KeyframeMotionIterator2::createArmTarget(int step, ArmTarget2 &arm_tgt) {
    auto next_kf = std::find_if(motion.keyframe.begin(), motion.keyframe.end(), [&](const KeyFrameInfo2 &kf) {
        return kf.step >= step;
    });

    //キーフレームを全て消化し終わっている場合
    if(next_kf == motion.keyframe.end()){
        return false;
    }

    arm_tgt.step_end = step+1;

    //最初のキーフレームに到達していない場合
    if (next_kf == motion.keyframe.begin()) {
        for (size_t idx = 0; idx < motion.jnames.size(); ++idx) {
            auto jname = motion.jnames[idx];
            JointState2 jstate;
            jstate.pos = next_kf->state[idx].pos;
            jstate.vel = 0;
            jstate.acc = 0;
            arm_tgt.tgt_state.emplace(jname, jstate);
        }
        return true;
    }



    auto prev_kf = next_kf;
    prev_kf--;

    for(size_t idx = 0;idx < motion.jnames.size();++idx){
        auto jname = motion.jnames[idx];
        double diff_sec = (next_kf->step - prev_kf->step) * motion.dt;
        double t1 = std::pow(diff_sec,1);
        double t2 = std::pow(diff_sec,2);
        double t3 = std::pow(diff_sec,3);
        double t4 = std::pow(diff_sec,4);
        double t5 = std::pow(diff_sec,5);

        double pos0 = prev_kf->state[idx].pos;
        double vel0 = prev_kf->state[idx].vel;
        double acc0 = prev_kf->state[idx].acc;

        double pos1 = next_kf->state[idx].pos;
        double vel1 = next_kf->state[idx].vel;
        double acc1 = next_kf->state[idx].acc;


        Eigen::Matrix3d mat;
        mat<<   t3,    t4,    t5,
              3*t2,  4*t3,  5*t4,
              6*t1, 12*t2, 20*t3;

        Eigen::Vector3d vq;
        vq << pos1 - pos0 - vel0 * t1 - 0.5 * acc0 * t2,
              vel1 - vel0 - acc0 * t1,
              acc1 - acc0;

        Eigen::Vector3d va = mat.inverse() * vq;

        double cur_sec = (step - prev_kf->step) * motion.dt;
        double jpos = pos0
                + vel0       * cur_sec
                + 0.5 * acc0 * std::pow(cur_sec, 2)
                + va(0)      * std::pow(cur_sec, 3)
                + va(1)      * std::pow(cur_sec, 4)
                + va(2)      * std::pow(cur_sec, 5);

        double jvel = vel0
                + acc0           * cur_sec
                + va(0)      * 3 * std::pow(cur_sec, 2)
                + va(1)      * 4 * std::pow(cur_sec, 3)
                + va(2)      * 5 * std::pow(cur_sec, 4);

        double jacc =  acc0
                + va(0)      * 6  *  cur_sec
                + va(1)      * 12 * std::pow(cur_sec, 2)
                + va(2)      * 20 * std::pow(cur_sec, 3);

        JointState2 state;
        state.pos = jpos;
        state.vel = jvel;
        state.acc = jacc;
        arm_tgt.tgt_state.emplace(jname, state);
    }

    return true;
}


TargetData2 KeyframeMotionIterator2::getValue() {


    TargetData2 target;
    target.dt = motion.dt;

    ArmTarget2 arm_tgt;
    if (createArmTarget(step, arm_tgt)) {
        target.arm_tgt.push_back(arm_tgt);
    }

    return target;
}

bool KeyframeMotionIterator2::seek(int step){
    return true;
}

bool KeyframeMotionIterator2::nextstep(){
    return true;
}


}
