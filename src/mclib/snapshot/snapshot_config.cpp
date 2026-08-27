// mclib
#include "mclib/snapshot/snapshot_config.hpp"

#include "mclib/control/robot_state.hpp"
#include "mclib/utils.hpp"

#include <algorithm>
#include <cmath>

namespace snapshot
{
  namespace
  {

    // Default local devices
    // Replace with extern declarations if your code has ownership of these sensors elsewhere
    mclib::device::Distance distFront(1);
    mclib::device::Distance distRight(2);

    SnapshotConfig g_cfg{};
    std::vector<DistanceSensorConfig> g_sensors;
    SnapshotPoseRuntime g_runtime{};

    bool g_init = false;
    bool g_cfg_custom = false;
    bool g_sensors_custom = false;

    bool runtime_ready(const SnapshotPoseRuntime &runtime)
    {
      if (runtime.apply_pose == nullptr)
        return false;
      if (runtime.get_pose != nullptr)
        return true;
      return runtime.get_heading_deg != nullptr &&
             runtime.get_guess_x_in != nullptr &&
             runtime.get_guess_y_in != nullptr;
    }

    // One read of the whole pose. get_pose() is the consistent path; the three
    // scalar getters are three separate reads and can straddle two odometry
    // ticks, so they are only the fallback for a runtime that has no better
    // option.
    void read_guess(const SnapshotPoseRuntime &runtime,
                    float &x_in, float &y_in, float &heading_deg)
    {
      if (runtime.get_pose != nullptr)
      {
        runtime.get_pose(runtime.user_data, &x_in, &y_in, &heading_deg);
        return;
      }
      heading_deg = runtime.get_heading_deg(runtime.user_data);
      x_in = runtime.get_guess_x_in(runtime.user_data);
      y_in = runtime.get_guess_y_in(runtime.user_data);
    }

    void apply_quadrant_bias(float &guess_x, float &guess_y, Quadrant q, float margin_in)
    {
      if (q == Quadrant::ANY)
        return;

      const float mid = FIELD_SIZE_IN * 0.5f;
      const float margin = std::max(0.0f, margin_in);

      switch (q)
      {
      case Quadrant::BL:
        guess_x = std::min(guess_x, mid - margin);
        guess_y = std::min(guess_y, mid - margin);
        break;
      case Quadrant::BR:
        guess_x = std::max(guess_x, mid + margin);
        guess_y = std::min(guess_y, mid - margin);
        break;
      case Quadrant::TL:
        guess_x = std::min(guess_x, mid - margin);
        guess_y = std::max(guess_y, mid + margin);
        break;
      case Quadrant::TR:
        guess_x = std::max(guess_x, mid + margin);
        guess_y = std::max(guess_y, mid + margin);
        break;
      case Quadrant::ANY:
      default:
        break;
      }
    }

    void init_once()
    {
      if (g_init)
        return;
      g_init = true;

      // 1) Sensor location on robot (per sensor):
      //      x_right_in: +right from robot center (in)
      //      y_fwd_in:   +forward from robot center (in)
      //      rel_deg:    facing relative to robot forward
      //                  0=forward, +90=right, 180=back, -90=left
      //
      // 2) Collidable map objects per sensor:
      //      field_mask_override = bitmask of allowed map objects for THAT sensor raycast.
      //      Use 0 to fall back to g_cfg.field_mask.
      //
      //    Mask options (from collision_map.hpp):
      //      MAP_PERIMETER
      //      MAP_LONG_GOALS
      //      MAP_LONG_GOAL_BRACES
      //      MAP_LONG_GOALS_ALL
      //      MAP_CENTER_GOAL_POS45
      //      MAP_CENTER_GOAL_NEG45
      //      MAP_CENTER_GOALS
      //      MAP_MATCHLOADERS
      //      MAP_PARK_ZONES
      //
      //    Example:
      //      front.field_mask_override = MAP_PERIMETER; // low sensor, walls only
      //      right.field_mask_override = MAP_PERIMETER | MAP_LONG_GOALS_ALL;

      if (!g_cfg_custom)
      {
        g_cfg = SnapshotConfig{};
        g_cfg.field_mask = MAP_PERIMETER;
        g_cfg.samples = 5;
        g_cfg.sample_delay_ms = 35;
        g_cfg.min_sensors = 2;
        g_cfg.max_chi2_per_sensor = 9.0f;
        g_cfg.max_correction_in = 12.0f;
        g_cfg.quadrant_margin_in = 2.0f;
      }

      if (!g_sensors_custom)
      {
        g_sensors.clear();
        g_sensors.reserve(2);

        DistanceSensorConfig front{};
        front.dev = &distFront;
        front.x_right_in = 0.0f;
        front.y_fwd_in = 7.0f;
        front.rel_deg = 0.0f;
        front.field_mask_override = MAP_PERIMETER;
        front.use_confidence_gate = true;
        front.min_confidence = 35;
        g_sensors.push_back(front);

        DistanceSensorConfig right{};
        right.dev = &distRight;
        right.x_right_in = 7.0f;
        right.y_fwd_in = 0.0f;
        right.rel_deg = 90.0f;
        right.field_mask_override = MAP_PERIMETER | MAP_LONG_GOALS_ALL;
        right.use_confidence_gate = true;
        right.min_confidence = 35;
        g_sensors.push_back(right);
      }

    }

    /**
     * @brief Applies a correction, but only if it is a small move from the
     *        pose that is actually being overwritten.
     *
     * The solver measures `correction_in` from the seed it was given, and for
     * a quadrant snapshot that seed has already been pulled toward the
     * quadrant by apply_quadrant_bias(). Gating there would bound the move
     * from the biased seed rather than from the odometry pose, so a 10 in
     * clamp plus a 12 in solve would write a 22 in jump under a 12 in cap.
     * This second gate closes that: it measures from the unbiased pose.
     */
    struct RuntimeOdomAdapter
    {
      SnapshotPoseRuntime runtime;
      float odom_x_in = 0.0f;
      float odom_y_in = 0.0f;
      float max_correction_in = 0.0f;
      bool applied = false;
      float correction_in = 0.0f;

      void set_position(float x_in,
                        float y_in,
                        float heading_deg,
                        float forward_tracker_in,
                        float sideways_tracker_in)
      {
        const float dx = x_in - odom_x_in;
        const float dy = y_in - odom_y_in;
        correction_in = std::sqrt(dx * dx + dy * dy);
        if (max_correction_in > 0.0f && correction_in > max_correction_in)
          return;

        runtime.apply_pose(runtime.user_data,
                           x_in,
                           y_in,
                           heading_deg,
                           forward_tracker_in,
                           sideways_tracker_in);
        applied = true;
      }
    };

  } // namespace

  bool snapshot_config_set_runtime(const SnapshotPoseRuntime &runtime)
  {
    if (!runtime_ready(runtime))
      return false;
    g_runtime = runtime;
    return true;
  }

  bool snapshot_config_use_robot_state()
  {
    SnapshotPoseRuntime runtime{};
    runtime.user_data = nullptr;
    runtime.get_pose = [](void *, float *x_in, float *y_in, float *heading_deg)
    {
      const mclib::Pose2D pose = mclib::control::robotState().pose();
      *x_in = static_cast<float>(pose.x);
      *y_in = static_cast<float>(pose.y);
      *heading_deg = static_cast<float>(radToDeg(pose.theta));
    };
    // The snapshot never solves for heading, so it writes position only and
    // leaves the IMU's theta in place - the same split as wallReset().
    runtime.apply_pose = [](void *, float x_in, float y_in, float, float, float)
    {
      mclib::control::robotState().setPosition(static_cast<double>(x_in),
                                               static_cast<double>(y_in));
    };
    g_runtime = runtime;
    return true;
  }

  void snapshot_config_set_sensors(const std::vector<DistanceSensorConfig> &sensors)
  {
    g_sensors = sensors;
    g_sensors_custom = true;
    g_init = false;
  }

  bool snapshot_config_update_sensor(std::size_t index, const DistanceSensorConfig &sensor)
  {
    init_once();
    if (index >= g_sensors.size())
      return false;
    g_sensors[index] = sensor;
    g_sensors_custom = true;
    return true;
  }

  bool snapshot_config_set_sensor_mask(std::size_t index, std::uint32_t mask_override)
  {
    init_once();
    if (index >= g_sensors.size())
      return false;
    g_sensors[index].field_mask_override = mask_override;
    g_sensors_custom = true;
    return true;
  }

  void snapshot_config_set_config(const SnapshotConfig &cfg)
  {
    g_cfg = cfg;
    g_cfg_custom = true;
    g_init = false;
  }

  void snapshot_config_reset_defaults()
  {
    g_cfg_custom = false;
    g_sensors_custom = false;
    g_cfg = SnapshotConfig{};
    g_sensors.clear();
    g_init = false;
  }

  SnapshotResult snapshot_setpose_quadrant(Quadrant q)
  {
    init_once();

    if (!runtime_ready(g_runtime))
    {
      SnapshotResult result{};
      result.reject = SnapshotReject::NO_RUNTIME;
      return result;
    }

    float odom_x = 0.0f;
    float odom_y = 0.0f;
    float heading_deg = 0.0f;
    read_guess(g_runtime, odom_x, odom_y, heading_deg);

    const float fwd_in = g_runtime.get_forward_tracker_in != nullptr
                             ? g_runtime.get_forward_tracker_in(g_runtime.user_data)
                             : 0.0f;
    const float side_in = g_runtime.get_sideways_tracker_in != nullptr
                              ? g_runtime.get_sideways_tracker_in(g_runtime.user_data)
                              : 0.0f;

    float seed_x = odom_x;
    float seed_y = odom_y;
    apply_quadrant_bias(seed_x, seed_y, q, g_cfg.quadrant_margin_in);

    RuntimeOdomAdapter odom{};
    odom.runtime = g_runtime;
    odom.odom_x_in = odom_x;
    odom.odom_y_in = odom_y;
    odom.max_correction_in = g_cfg.max_correction_in;

    SnapshotResult result = snapshot_setpose(odom,
                                             g_sensors,
                                             g_cfg,
                                             heading_deg,
                                             fwd_in,
                                             side_in,
                                             seed_x,
                                             seed_y);

    if (result.success)
    {
      // Report the move from the odometry pose, not from the biased seed.
      result.correction_in = odom.correction_in;
      if (!odom.applied)
      {
        result.success = false;
        result.reject = SnapshotReject::CORRECTION_TOO_LARGE;
        result.x_in = odom_x;
        result.y_in = odom_y;
      }
    }
    return result;
  }

} // namespace snapshot
