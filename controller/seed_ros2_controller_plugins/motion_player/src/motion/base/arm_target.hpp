#pragma once

#include "common.hpp"
#include "joint_state.hpp"

namespace motion_editor2 {

struct ArmTarget2 {
    std::map<std::string, JointState2> tgt_state;
    int step_end;
};

}
