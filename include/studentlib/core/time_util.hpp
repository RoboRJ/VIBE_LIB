#pragma once

#include "studentlib/core/types.hpp"

namespace studentlib {

timestamp_ms_t nowMilliseconds();
double nowSeconds();

void delayMilliseconds(timestamp_ms_t delay_ms);
void delaySeconds(double delay_seconds);

bool hasElapsed(timestamp_ms_t start_time_ms, timestamp_ms_t duration_ms);

}  // namespace studentlib