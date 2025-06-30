#pragma once

#include <atomic>

namespace rtlogger{

template<class T>
class Singleton {
public:
    static T& getInstance() {
        if (!instance) {
            create();
        }
        return *instance;
    }

    static void destroy() {
        delete instance;
        instance = nullptr;
        is_finished.store(true);
    }

    static bool finished(){
        return is_finished.load();
    }

private:
    static void create() {
        instance = new T();
    }

    Singleton() = delete;
    ~Singleton() = delete;
private:
    static T *instance;
    static std::atomic<bool> is_finished;
};

template<class T> std::atomic<bool> Singleton<T>::is_finished = false;
template<class T> T* Singleton<T>::instance = nullptr;
}

template<class T>
class SingletonHolder {
public:
    SingletonHolder() {
        rtlogger::Singleton<T>::getInstance();
    }

    ~SingletonHolder() {
        rtlogger::Singleton<T>::destroy();
    }
};

