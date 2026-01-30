#pragma once

#include "common.hpp"
#include "fileio_json_input_archive.hpp"
#include "type_traits.hpp"

void fromJson(picojson::value json, int &value);
void fromJson(picojson::value json, unsigned int &value);
void fromJson(picojson::value json, short unsigned int &value);
void fromJson(picojson::value json, char &value);
void fromJson(picojson::value json, unsigned char &value);
void fromJson(picojson::value json, bool &value);
void fromJson(picojson::value json, double &value);
void fromJson(picojson::value json, std::string &value);

template<int N>
void fromJson(picojson::value json, char (&value)[N]){
    strncpy(value,json.get<std::string>().c_str(),N);
}

//pairの場合は、object要素として["キー":値]となっている
template<class T>
void fromJson(picojson::value json, std::pair<std::string, T> &value) {
    if(!json.is<picojson::object>()) {
        return;
    }

    auto obj = json.get<picojson::object>();
    if(obj.size() != 1){
        return;
    }

    value.first = obj.begin()->first;
    fromJson(obj.begin()->second, value.second);
}

//mapの場合は、object要素として保存されている場合と、array要素として保存されている場合がある。
template<class T>
void fromJson(picojson::value json, std::map<std::string, T> &value) {

    value.clear();
    if (json.is<picojson::object>()) {//["キー":値,"キー":値]となっている場合
        auto obj = json.get<picojson::object>();
        for (auto elem : obj) {
            std::string key = elem.first;
            T tmp;
            fromJson(elem.second, tmp);
            value.emplace(key, tmp);
        }
    } else if (json.is<picojson::array>()) {//[{"キー":値},{"キー":値}]となっている場合
        auto array = json.get<picojson::array>();
        for (auto &elem : array) {
            std::pair<std::string,T> tmp;
            fromJson(elem, tmp);
            value.emplace(tmp);
        }
    }
}

//イテレータを持たない要素の場合
template<class T>
auto fromJson(picojson::value json, T &value) -> typename std::disable_if<has_iterator<T>::value>::type{
    if(json.is<picojson::null>()){
        return;
    }

    JsonInputArchive archive(json);
    archive >> value;
}

//イテレータを持っている要素の場合の処理
//可変長なのは、vectorとかが、アロケータが第２テンプレート引数となっているため
template<template<class...> class T,class U>
auto fromJson(picojson::value json, T<U> &value) -> typename std::enable_if<has_iterator<T<U>>::value & !std::is_pointer<U>::value>::type {

    value.clear();
    if(!json.is<picojson::array>()) {
        //要素が一つの場合
        U tmp;
        fromJson(json,tmp);
        value.push_back(tmp);
    } else if(json.is<picojson::array>()) {
        auto array = json.get<picojson::array>();
        for (auto &elem : array) {
            U tmp;
            fromJson(elem, tmp);
            value.push_back(tmp);
        }
    }
}

//イテレータを持っている要素の場合の処理
//可変長なのは、vectorとかが、アロケータが第２テンプレート引数となっているため
template<template<class...> class T,class U>
auto fromJson(picojson::value json, T<U> &value) -> typename std::enable_if<has_iterator<T<U>>::value & std::is_pointer<U>::value>::type {

    value.clear();
    if(!json.is<picojson::array>()) {
        //要素が一つの場合
        U tmp = nullptr;
        fromJson(json,tmp);
        value.push_back(tmp);
    } else if(json.is<picojson::array>()) {
        auto array = json.get<picojson::array>();
        for (auto &elem : array) {
            U tmp = nullptr;
            fromJson(elem, tmp);
            value.push_back(tmp);
        }
    }
}

//配列の場合
template<class T,int N>
auto fromJson(picojson::value json, T(&value)[N]) ->typename std::disable_if<std::is_same<T, char>::value>::type{
    auto array = json.get<picojson::array>();
    int idx = 0;
    for (auto &elem : array) {
        T tmp;
        fromJson(elem, tmp);
        value[idx++] = tmp;
    }
}



template<class T>
auto fromJson(picojson::value json, const T &value) {
    fromJson(json, const_cast<T&>(value));
}

template<class T>
void fromJson(picojson::value json, KeyedValue<T>& keyedval){
    if(!json.is<picojson::object>()){
        return;
    }
    auto obj = json.get<picojson::object>();
    if(obj.count(keyedval.key) == 0){
        return;
    }

    picojson::value val = json.get<picojson::object>()[keyedval.key];
    fromJson(val,keyedval.value);
}


template<class T>
void operator>>(JsonFormat& json, KeyedValue<T>& keyedval){
    if(json.obj.count(keyedval.key) == 0){
        return;
    }
    picojson::value val = json.obj[keyedval.key];
    fromJson(val,keyedval.value);
}
