#pragma once

#include "common.hpp"
#include <QtWidgets>

template<class T1, class T2, class ... Args>
void signalForwarding(T1 *ptr1, void (T1::*func1)(Args...), T2 *ptr2, void (T2::*func2)(Args...)) {
    QObject::connect(ptr1, func1, ptr2, [=](Args ... args) {
        Q_EMIT (ptr2->*func2)(args...);
    });
}

Q_DECLARE_METATYPE(std::string)
