#include "keyframe_motion2.hpp"
#include "keyframe_motion_iterator.hpp"
#include <Eigen/Dense>
#include <Eigen/LU>

namespace motion_editor2 {

MotionIterator2::Ptr KeyframeMotion2::getIterator() const {
    return std::make_shared<KeyframeMotionIterator2>(*this);
}

}
