#pragma once

#include <chrono>

namespace simulation {

using Duration = std::chrono::steady_clock::duration;
using TimePoint = std::chrono::steady_clock::time_point;

}
