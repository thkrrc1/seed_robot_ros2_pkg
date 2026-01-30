#pragma once

#include "common.hpp"

namespace motion_editor2 {

class ContainerBase {
public:
    virtual ~ContainerBase(){

    }

    virtual ContainerBase* clone(){
        return nullptr;
    }
};

template<class T>
class Container: public ContainerBase {
public:
    Container(const T &data) {
        this->data = data;
    }

    ContainerBase* clone(){
        return new Container<T>(data);
    }

    T data;
};

class ExtensionInfo2 {
public:
    enum {TYPE_INVALID=-1};
    ExtensionInfo2(const ExtensionInfo2& info){
        type = info.type;
        container = info.container->clone();
    }

    template<class T>
    ExtensionInfo2(const T &data) {
        container = new Container<T>(data);
        type = T::type();
    }


    int get_type() {
        return type;
    }

    ~ExtensionInfo2() {
        delete container;
    }

    template<class T>
    T* as() {
        if (get_type() == T::type()) {
            return &static_cast<Container<T>*>(container)->data;
        }
        return nullptr;
    }
private:
    int type = TYPE_INVALID;
    ContainerBase *container = nullptr;
};

}
