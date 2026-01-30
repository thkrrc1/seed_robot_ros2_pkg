#pragma once

#include "common.hpp"
#include "fileio.hpp"

struct JointState2{
    double pos = 0;
    double vel = 0;
    double acc = 0;

protected:
    friend class ::access;
    template<class T>
    void serialize(T &ar) {
        ar & ARCHIVE_NAMEDVALUE(pos);
        ar & ARCHIVE_NAMEDVALUE(vel);
        ar & ARCHIVE_NAMEDVALUE(acc);
    }
};
