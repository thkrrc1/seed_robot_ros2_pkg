#pragma once

#include "common.hpp"
#include "target_data.hpp"
#include "media_info.hpp"

namespace motion_editor2 {

/**
 * モーションを、再生可能な形で取得するためのイテレータ
 */
class MotionIterator2 {
public:
    MotionIterator2() {
    }
    virtual ~MotionIterator2() {
    }

    //! 全ステップ数
    virtual int getMaxStep() = 0;

    //! 一ステップの時間
    virtual double getStepTime() = 0;

    //! そのステップに値があるか
    virtual bool hasElement(int step) = 0;

    virtual bool seek(int step) {
        return false;
    }

    virtual bool nextstep() {
        return false;
    }

    //! 値が準備できるか
    virtual bool valueAvailable() {
        return true;
    }

    //! 値の取得
    virtual TargetData2 getValue() = 0;

    typedef std::shared_ptr<MotionIterator2> Ptr;
};

}
