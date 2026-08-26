// mclib
#pragma once

#include "mclib/snapshot/snapshot.hpp"

#include <cstddef>
#include <cstdint>
#include <vector>

namespace snapshot {

bool snapshot_config_set_runtime(const SnapshotPoseRuntime& runtime);
void snapshot_config_set_sensors(const std::vector<DistanceSensorConfig>& sensors);
bool snapshot_config_update_sensor(std::size_t index,
                                   const DistanceSensorConfig& sensor);
bool snapshot_config_set_sensor_mask(std::size_t index,
                                     std::uint32_t mask_override);
void snapshot_config_set_config(const SnapshotConfig& cfg);
void snapshot_config_reset_defaults();
SnapshotResult snapshot_setpose_quadrant(Quadrant q);

}  // namespace snapshot
