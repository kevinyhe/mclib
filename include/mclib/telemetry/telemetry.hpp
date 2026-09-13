// mclib
//
// This Source Code Form is subject to the terms of the Mozilla Public
// License, v. 2.0. If a copy of the MPL was not distributed with this
// file, You can obtain one at https://mozilla.org/MPL/2.0/.
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
