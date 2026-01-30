#pragma once

#ifdef WIN32
#include <winsock2.h>
#define _USE_MATH_DEFINES
#include <math.h>
#endif

#include<cmath>
#include <iostream>
#include <fstream>
#include <sstream>
#include <list>
#include <vector>
#include <algorithm>
#include <string>
#include <thread>
#include <chrono>
#include <regex>
#include <memory>
#include <mutex>
#include <map>
#include <unordered_map>
#include <typeinfo>
#include <queue>
#include <condition_variable>
#include <typeindex>
#include <sys/types.h>
#include <sys/wait.h>
#include <unistd.h>
#include <pthread.h>
#include <cstdlib>
#include <cstring>
#include <atomic>

#if __GNUC__ <= 7
#include <experimental/filesystem>

namespace std {
namespace experimental {
namespace filesystem {
path relative(path p, path base);
}
}
namespace filesystem = experimental::filesystem;
}

#else
#include <filesystem>
#endif

std::string getTimeStamp();
#include "rt_logger/logger.hpp"
