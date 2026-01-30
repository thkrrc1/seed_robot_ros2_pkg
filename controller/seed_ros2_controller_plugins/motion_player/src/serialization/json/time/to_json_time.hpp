#pragma once

#include "common.hpp"
#include "fileio_json_output_archive.hpp"
#include "type_traits.hpp"

picojson::value toJson(std::chrono::system_clock::time_point &value);
