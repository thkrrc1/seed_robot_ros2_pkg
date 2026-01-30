#pragma once

#include <QPixmap>

#include "common.hpp"
#include "keyframe_motion2.hpp"


namespace motion_editor2{
class KeyframeMotionIterator2: public MotionIterator2 {

public:
    KeyframeMotionIterator2(const KeyframeMotion2 &motion);

    ~KeyframeMotionIterator2();

    double getStepTime() override;

    int getMaxStep() override;

    bool hasElement(int step) override;

    bool valueAvailable() override {
        return true;
    }

    bool seek(int step) override;

    bool nextstep() override;

    TargetData2 getValue() override;

private:
    bool createArmTarget(int step,ArmTarget2& arm_tgt);

private:
    int step = 0; //!< 現在のステップ
    const KeyframeMotion2 &motion; //!< 元のモーション
};
}
