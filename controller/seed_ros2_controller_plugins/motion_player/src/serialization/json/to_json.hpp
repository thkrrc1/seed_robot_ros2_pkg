#pragma once

#include "common.hpp"
#include "fileio_json_output_archive.hpp"
#include "type_traits.hpp"

picojson::value toJson(const int &value);
picojson::value toJson(const unsigned int &value);
picojson::value toJson(const short unsigned int & value);
picojson::value toJson(const unsigned char &value);
picojson::value toJson(const char &value);
picojson::value toJson(const bool &value);
picojson::value toJson(const double &value);
picojson::value toJson(const std::string &value);
picojson::value toJson(const picojson::array &value);

template<int N>
picojson::value toJson(const char (&value)[N]){
    return picojson::value(value);
}

//pairの場合、{"キー",値}として保存する
template<class T>
picojson::value toJson(const std::pair<std::string, T> &value) {
    picojson::object obj;
    picojson::value val = toJson(value.second);//値の部分をJson化
    if (!val.is<picojson::null>()) {
        obj.emplace(std::make_pair(value.first, val));//キーとセットにする
        return picojson::value(obj);
    } else {
        return picojson::value();
    }
}

template<class T>
auto toJson(const T &value) -> typename std::disable_if<has_iterator<T>::value,picojson::value>::type {
    JsonOutputArchive archive(false);
    archive << value;
    return archive.get_json_value();
}

//イテレータのあるアイテムの場合、[値,値,・・]として保存する
template<class T>
auto toJson(const T &value) -> typename std::enable_if<has_iterator<T>::value,picojson::value>::type {

    picojson::array array;
    for (auto val : value) {
        auto jsonval = toJson(val);//値の部分をJson化
        array.push_back(jsonval);//リストに入れていく
    }
    if (!array.empty()) {
        return picojson::value(array);
    } else {
        return picojson::value();
    }
}

//配列の場合、[値,値,・・]として保存する
template<class T,int N>
auto toJson(const T(&value)[N])->typename std::disable_if<std::is_same<T, char>::value,picojson::value>::type {

    picojson::array array;
    for (auto itr = std::begin(value); itr != std::end(value); ++itr) {
        auto jsonval = toJson(*itr); //値の部分をJson化
        array.push_back(jsonval); //リストに入れていく
    }
    if (!array.empty()) {
        return picojson::value(array);
    } else {
        return picojson::value();
    }
}


//mapの場合、{"キー":値,"キー":値,・・]として保存する
//mapはkeyが一つだけなので、object型にできる
template<class T>
picojson::value toJson(const std::map<std::string, T> &value) {
    picojson::object obj;
    for (auto val : value) {
        obj.emplace(std::make_pair(val.first,toJson(val.second)));
    }

    if (!obj.empty()) {
        return picojson::value(obj);
    } else {
        return picojson::value();
    }
}

template<class T>
picojson::value toJson(const KeyedValue<T> &keyedval) {
    picojson::object obj;
    picojson::value value = toJson(keyedval.value);

    if (!value.is<picojson::null>()) {
        obj.emplace(std::make_pair(keyedval.key, value));
    }
    return picojson::value(obj);
}


template<class T>
void operator<<(JsonFormat &json, const KeyedValue<T> &keyedval) {
    picojson::value value = toJson(keyedval.value);
    if (!value.is<picojson::null>()) {
        json.obj.emplace(std::make_pair(keyedval.key, value));
    }
}
