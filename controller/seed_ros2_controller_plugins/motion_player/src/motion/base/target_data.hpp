#pragma once

#include "common.hpp"
#include "arm_target.hpp"
#include "extension_target.hpp"

namespace motion_editor2{

struct TargetData2{
        double dt; //ステップ時間[s]
        std::vector<ArmTarget2> arm_tgt;
        std::vector<ExtensionInfo2> extension;
        TargetData2(){
            dt = 0.02;
        }
};
}
