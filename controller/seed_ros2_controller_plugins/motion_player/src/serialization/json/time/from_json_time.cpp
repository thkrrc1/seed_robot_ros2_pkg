#include "from_json_time.hpp"

//各型から、picojsonへの変換
void fromJson(picojson::value json, std::chrono::system_clock::time_point &value){
    auto obj = json.get<picojson::object>();
    auto sec = static_cast<int>(obj["sec"].get<double>());
    auto nsec = static_cast<int>(obj["nsec"].get<double>());
    auto nsec_all = 1000000000UL * sec + nsec;
    value = std::chrono::time_point<std::chrono::system_clock>(std::chrono::nanoseconds(nsec_all));
}
