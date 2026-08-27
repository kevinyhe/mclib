// mclib
#pragma once

/**
 * @brief Umbrella header for the telemetry subsystem.
 *
 * @details Include this to get the logger, the sinks and the background flush
 * task. flush_task.hpp is the only piece backed by a PROS translation unit;
 * everything else is header-only and compiles on the host.
 */

#include "mclib/telemetry/flush_task.hpp"
#include "mclib/telemetry/logger.hpp"
#include "mclib/telemetry/sd_sink.hpp"
#include "mclib/telemetry/sink.hpp"
