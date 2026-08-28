// mclib
#pragma once

#include "mclib/snapshot/snapshot.hpp"

#include <cstddef>
#include <cstdint>
#include <vector>

namespace snapshot {

/**
 * @brief How many configured sensors are actually reporting on their port.
 *
 * @details The one call that tells a wrong port apart from a wall that is out
 * of range. A distance sensor on an empty or mistyped port makes PROS return
 * PROS_ERR (INT32_MAX), which the millimetre and inch accessors hand back as
 * ~84 million inches - a number no range check catches, and `get_confidence()`
 * fails the same way, so the confidence gate does not save you either. This
 * goes through `Distance::distance()`, which reports the bad port instead.
 *
 * A snapshot needs `SnapshotConfig::min_sensors` live sensors to solve at all.
 * If this returns fewer than that, `snapshot_setpose_quadrant()` will answer
 * `TOO_FEW_SENSORS` on every call and the ports are the reason. Check it once
 * at initialise, not inside the autonomous.
 *
 * @return Count of sensors whose port is reporting a distance sensor. Sensors
 *         with a null `dev` never count.
 */
std::size_t snapshot_config_live_sensor_count();

/// @brief How many sensors are configured, live or not. Pairs with
///        `snapshot_config_live_sensor_count()`.
std::size_t snapshot_config_sensor_count();

bool snapshot_config_set_runtime(const SnapshotPoseRuntime& runtime);

/**
 * @brief Install a runtime backed by `mclib::control::robotState()`.
 *
 * `robotState()` is the single pose source since Phase 3 - the odometry task
 * writes it, every motion routine reads it - so this is what a snapshot should
 * normally correct. The vtable seam is kept so the solver still does not know
 * that type exists.
 *
 * The tracker getters return 0: `RobotState` has no tracker offsets, and the
 * pose it holds is already at the tracking centre.
 *
 * @return True. Provided for symmetry with `snapshot_config_set_runtime()`.
 */
bool snapshot_config_use_robot_state();

void snapshot_config_set_sensors(const std::vector<DistanceSensorConfig>& sensors);
bool snapshot_config_update_sensor(std::size_t index,
                                   const DistanceSensorConfig& sensor);
bool snapshot_config_set_sensor_mask(std::size_t index,
                                     std::uint32_t mask_override);
void snapshot_config_set_config(const SnapshotConfig& cfg);
void snapshot_config_reset_defaults();

/**
 * @brief Take a snapshot, biasing the guess into a known quadrant first.
 *
 * @return The solve and its verdict. `result.success` is false and
 *         `result.reject` says why when the correction was refused; the pose
 *         is untouched in that case.
 */
SnapshotResult snapshot_setpose_quadrant(Quadrant q);

}  // namespace snapshot
