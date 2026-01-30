#pragma once

#include "common.hpp"
#include "fileio_json_input_archive.hpp"
#include "type_traits.hpp"

void fromJson(picojson::value json, std::chrono::system_clock::time_point &value);
