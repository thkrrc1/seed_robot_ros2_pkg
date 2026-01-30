#include "from_json.hpp"

//各型から、picojsonへの変換

void fromJson(picojson::value json, int &value) {
    if (!json.is<picojson::null>()) {
        value = static_cast<int>(json.get<double>());
    } else {
        value = 0;
    }
}

void fromJson(picojson::value json, unsigned int &value) {
    if (!json.is<picojson::null>()) {
        value = static_cast<unsigned int>(json.get<double>());
    } else {
        value = 0;
    }
}

void fromJson(picojson::value json, short unsigned int &value) {
    if (!json.is<picojson::null>()) {
        value = static_cast<short unsigned int>(json.get<double>());
    } else {
        value = 0;
    }
}

void fromJson(picojson::value json, char &value) {
    if (!json.is<picojson::null>()) {
        value = static_cast<char>(json.get<double>());
    } else {
        value = 0;
    }
}

void fromJson(picojson::value json, unsigned char &value) {
    if (!json.is<picojson::null>()) {
        value = static_cast<unsigned char>(json.get<double>());
    } else {
        value = 0;
    }
}

void fromJson(picojson::value json, bool &value) {
    if (!json.is<picojson::null>()) {
        value = json.get<bool>();
    } else {
        value = 0;
    }
}

void fromJson(picojson::value json, double &value) {
    if (!json.is<picojson::null>()) {
        value = json.get<double>();
    } else {
        value = 0;
    }
}

void fromJson(picojson::value json, std::string &value) {
    if (!json.is<picojson::null>()) {
        value = json.get<std::string>();
    } else {
        value = "";
    }
}

