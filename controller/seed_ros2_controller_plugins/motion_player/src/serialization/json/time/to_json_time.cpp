#include "to_json_time.hpp"

picojson::value toJson(std::chrono::system_clock::time_point &value){
    int32_t sec = std::chrono::duration_cast<std::chrono::seconds>(value.time_since_epoch()).count();
    int32_t nsec = std::chrono::duration_cast<std::chrono::nanoseconds>(value.time_since_epoch()).count() % 1000000000UL;
    picojson::object obj;
    obj["sec"] = picojson::value(static_cast<double>(sec));
    obj["nsec"] = picojson::value(static_cast<double>(nsec));

    return picojson::value(obj);
}
