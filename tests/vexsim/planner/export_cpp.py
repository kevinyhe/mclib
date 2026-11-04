"""A routine as an mclib autonomous() in C++.

Paths use the code in path_helpers.hpp, copied into the output, so the robot
bakes, plans and follows each path exactly as the simulator did. The lift
and the claw use cascade_helpers.hpp the same way."""
from pathlib import Path

from mechanisms import ACTION_TIMES, MECHANISM_ENUMS, MECHANISMS, PRELOADS, cpp_preset, describe_match_load
from geometry import designed_poses, path_layout
from project import MECHANISM_STEPS, PRESETS, STEP_TYPES, TRACKING, carries_speed, step_time_limit

HERE = Path(__file__).resolve().parent
PLACEHOLDER_FEEDFORWARD = dict(ks=0.7, kv=0.13, top_ips=60.0, track_in=13.5)


def number(value):
    text = f"{value:.4f}".rstrip("0").rstrip(".")
    return "0" if text in ("-0", "") else text


def inches(value):
    return f"{number(value)}_in"


def degrees(value):
    return f"{number(value)}_deg"


def millis(value):
    return f"{round(value)}_ms"


def volts(value):
    return f"{number(value)}_V"


def exported_helpers(name="path_helpers.hpp"):
    text = (HERE / name).read_text()
    begin, end = "// BEGIN EXPORTED HELPERS\n", "// END EXPORTED HELPERS"
    return text[text.index(begin) + len(begin):text.index(end)].rstrip() + "\n"


def uses_cascade(project):
    return project["preload"] != "none" or any(step["type"] in MECHANISM_STEPS or step["actions"]
                                               for step in project["steps"])


CASCADE_SETUP = """// The lift motors and the claw's valve. The ports are placeholders: set them
// to your robot's. The two lift motors share one shaft, so one is reversed.
// Before the match, with the lift all the way down, power on the robot so the
// encoders read 0. The claw's solenoid is on 3-wire port A; true closes it.
mclib::device::MotorGroup lift_motors({7, -8}, mclib::device::Gearset::Green);
mclib::device::Pneumatic claw_piston('A');
cascade::Rig cascade_rig([] { return lift_motors.getAveragePositionDeg(); },
                         [](double volts) { lift_motors.setVoltage(volts); },
                         [](bool closed) { claw_piston.set_value(closed); });
"""

CASCADE_TASK = """  // Runs the lift and the actions of each motion every 10 ms, as the
  // simulator did. It keeps holding the last lift preset after autonomous.
  static pros::Task cascade_task([] {
    while (true) {
      const auto pose = mclib::control::robotState().pose();
      cascade_rig.tick(pose.x, pose.y, mclib::time::millis());
      pros::delay(10);
    }
  });"""


def action_literal(action):
    when = "cascade::Action::When::" + {"start": "Start", "distance": "AfterDistance", "time": "AfterTime",
                                        "end": "End"}[action["when"]]
    preset = f"static_cast<int>({cpp_preset(action['mechanism'], action['preset'])})"
    return f"{{{when}, {number(action['value'])}, {MECHANISM_ENUMS[action['mechanism']]}, {preset}}}"


def with_actions(step, call, stop, result_type="MotionResult", reached="MotionResult::Reached"):
    """A motion call wrapped so the step's mechanism actions run during it."""
    literals = ", ".join(action_literal(action) for action in step["actions"])
    when = "; ".join(f"{MECHANISMS[action['mechanism']]['label']} to "
                     f"{MECHANISMS[action['mechanism']]['presets'][action['preset']]} "
                     f"{ACTION_TIMES[action['when']].lower()}"
                     + (f" {number(action['value'])} {'in' if action['when'] == 'distance' else 'ms'}"
                        if action["when"] in ("distance", "time") else "")
                     for action in step["actions"])
    check = (f"    if (result != {reached}) return;" if stop else "    (void)result;")
    return ["  {", f"    // During this motion: {when}.",
            "    const auto pose = mclib::control::robotState().pose();",
            f"    cascade_rig.beginActions({{{literals}}}, pose.x, pose.y, mclib::time::millis());",
            f"    const {result_type} result = {call};",
            "    cascade_rig.finishActions();", check, "  }"]


def lift_call(step):
    return f"cascade_rig.moveLift({cpp_preset('lift', step['lift_preset'])}, {step['timeout_ms']})"


def claw_call(step):
    return f"cascade_rig.setClaw({cpp_preset('claw', step['claw_preset'])})"


def match_load_comment(step):
    return (f"  // The Drive Team stands {describe_match_load(step)}. The robot does nothing. Only legal in the "
            "Driver Controlled Period (SG11a).")


def describe(step, index):
    label = STEP_TYPES[step["type"]]["label"]
    note = f": {step['note']}" if step.get("note") else ""
    return f"// {index + 1}. {label}{note}"


def exit_flag(step):
    return "false" if step["chain"] else "true"


def motion_call(step):
    kind, t, v = step["type"], step["timeout_ms"], step["volts"]
    direction = step.get("direction", 1)
    stop = exit_flag(step)
    if kind == "drive":
        return f"driveTo({inches(step['distance'])}, {millis(t)}, {stop}, {volts(v)})"
    if kind == "turn":
        return f"turnToAngle({degrees(step['heading'])}, {millis(t)}, {stop}, {volts(v)})"
    if kind == "point":
        return f"moveToPoint({inches(step['x'])}, {inches(step['y'])}, {direction}, {millis(t)}, {stop}, {volts(v)})"
    if kind == "turn_to_point":
        return f"turnToPoint({inches(step['x'])}, {inches(step['y'])}, {direction}, {millis(t)})"
    if kind == "boomerang":
        return (f"boomerang({inches(step['x'])}, {inches(step['y'])}, {direction}, {degrees(step['heading'])}, "
                f"{number(step['lead'])}, {millis(t)}, {stop}, {volts(v)})")
    if kind == "arc":
        return f"curveCircle({degrees(step['heading'])}, {inches(step['radius'])}, {millis(t)}, {stop}, {volts(v)})"
    if kind == "reverse_arc":
        return (f"curveCircleReverse({degrees(step['heading'])}, {inches(step['radius'])}, {millis(t)}, "
                f"{stop}, {volts(v)})")
    if kind == "swing":
        return f"swing({degrees(step['heading'])}, {direction}, {millis(t)}, {stop}, {volts(v)})"
    raise ValueError(f"no motion call for {kind}")


def export(project, settings=None, plans=None):
    """C++ for @p project. @p settings and @p plans come from a plan; without
    them the feedforward lines are placeholders to measure on the robot."""
    feedforward = settings["feedforward"] if settings else PLACEHOLDER_FEEDFORWARD
    defaults = project["path_defaults"]
    robot = project["robot"]
    start = project["start"]
    poses = designed_poses(project)
    has_path = any(step["type"] == "path" for step in project["steps"])
    cascade = uses_cascade(project)
    stop = project["stop_on_failure"]
    name = project["name"].replace("\"", "'")
    lines = [f"// \"{name}\", exported from the mclib path planner.",
             f"// Simulated on: {PRESETS[robot['preset']]['label']}; {TRACKING[robot['tracking']].lower()}.",
             "// Units: inches and degrees. 0 deg faces +Y (forward); clockwise is positive.",
             f"// Place the robot at ({number(start['x'])}, {number(start['y'])}) facing "
             f"{number(start['heading'])} deg before autonomous starts.",
             "#include \"mclib/mclib.hpp\"", ""]
    if has_path:
        lines += ["// From mclib tests/vexsim/planner/path_helpers.hpp (Mozilla Public License 2.0):",
                  "// bakes each path, plans its speeds and follows it, as the simulator did.",
                  exported_helpers()]
    if cascade:
        lines += ["// From mclib tests/vexsim/planner/cascade_helpers.hpp (Mozilla Public License 2.0):",
                  "// the lift presets, the claw, and the actions run during a motion, as the",
                  "// simulator ran them.",
                  exported_helpers("cascade_helpers.hpp"), CASCADE_SETUP]
    lines += ["using namespace mclib::units::literals;", "using mclib::control::MotionResult;", "",
              "void autonomous() {",
              "  mclib::control::DriveHardware* drive = mclib::control::requireDrive(\"autonomous\");",
              "  if (drive == nullptr) return;",
              f"  drive->setHeadingDeg({number(start['heading'])});",
              f"  mclib::control::resetOdometry({{{number(start['x'])}, {number(start['y'])}, "
              f"{number(start['heading'])} * mclib::kPi / 180}});",
              f"  mclib::control::robotState().setCorrectAngleDeg({number(start['heading'])});"]
    if cascade:
        if project["preload"] != "none":
            lines.append(f"  // Preload: {PRELOADS[project['preload']]}. It stays there when the robot drives away.")
        lines.append(CASCADE_TASK)
    if has_path:
        source = ("measured on the simulated robot. Measure your own robot (docs/motion.md)."
                  if settings else "placeholders. Measure them on your robot (docs/motion.md).")
        lines += ["", f"  // RAMSETE settings. kS, kV and the track width are {source}",
                  "  mclib::control::RamseteConfig follow;",
                  f"  follow.feedforward.kS = {volts(feedforward['ks'])};",
                  f"  follow.feedforward.kV = {number(feedforward['kv'])}_V / mclib::units::inps;",
                  f"  follow.track_width = {inches(feedforward['track_in'])};",
                  f"  follow.gains.b = {number(defaults['ramsete_b'])};",
                  f"  follow.gains.zeta = {number(defaults['ramsete_zeta'])};",
                  f"  const path_planner::Limits limits{{{number(defaults['accel_ips2'])}, "
                  f"{number(defaults['cornering_ips2'])}, {number(feedforward['track_in'])}}};"]

    def guarded(call):
        return f"  if ({call} != MotionResult::Reached) return;" if stop else f"  {call};"

    for index, step in enumerate(project["steps"]):
        lines += ["", "  " + describe(step, index)]
        if step["chain"]:
            lines.append("  // Keeps moving into the next step: exit = false, so it does not stop or settle.")
        if step["type"] == "path":
            if not step["points"]:
                lines.append("  // This path has no points yet.")
                continue
            plan = plans[index] if plans else None
            layout = path_layout(step, poses[index][0])
            timeout = step_time_limit(step, plan) if plan or step["timeout_ms"] is not None else 8000
            if plan:
                lines.append(f"  // Planned in the simulator: {plan['duration_s']:.2f} s, {plan['length_in']:.1f} in, "
                             f"top speed {plan['top_speed_ips']:.1f} in/s.")
            lines += ["  {", "    const auto plan = path_planner::planSections({"]
            for section in layout["sections"]:
                percent = section["speed_percent"] if section["speed_percent"] is not None else defaults["speed_percent"]
                speed = percent / 100 * feedforward["top_ips"]
                points = ", ".join(f"{{{number(p[0])}, {number(p[1])}}}" for p in
                                   (section["start"], section["start_handle"], section["end_handle"], section["end"]))
                lines.append(f"        {{{points}, {number(speed)}, {'true' if section['reverse'] else 'false'}}},")
            if carries_speed(project, index) and not plan:
                lines.append("  // The speed at the join into the next path is 0 until the planner has measured "
                             "the robot. Export again once it has.")
            join = (f", {number(plan['requested_start_ips'])}, {number(plan['requested_end_ips'])}"
                    if plan and (plan["requested_start_ips"] or plan["requested_end_ips"]) else "")
            follow_call = (f"path_planner::followSections(plan, follow, {millis(timeout)}, "
                           f"{volts(step['volts'])}, {exit_flag(step)})")
            lines.append(f"    }}, limits{join});")
            if step["actions"]:
                lines += ["  " + line for line in with_actions(step, follow_call, stop)]
            else:
                lines.append("  " + guarded(follow_call))
            lines.append("  }")
        elif step["type"] == "wall_reset":
            call = (f"wallReset({inches(step['x'])}, {inches(step['y'])}, {degrees(step['heading'])}, "
                    f"{volts(step['volts'])}, {millis(step['timeout_ms'])}, "
                    f"{number(step['current_ma'])} * mclib::units::milliampere)")
            if step["actions"]:
                lines += with_actions(step, call, stop, result_type="bool", reached="true")
            else:
                lines.append(f"  if (!{call}) return;" if stop else f"  {call};")
        elif step["type"] == "lift":
            call = lift_call(step)
            lines.append(f"  if ({call} == cascade::MoveResult::TimedOut) return;" if stop else f"  {call};")
        elif step["type"] == "claw":
            lines.append(f"  {claw_call(step)};")
        elif step["type"] == "match_load":
            lines.append(match_load_comment(step))
        elif step["type"] == "wait":
            lines.append(f"  pros::delay({step['wait_ms']});")
        elif step["actions"]:
            lines += with_actions(step, motion_call(step), stop)
        else:
            lines.append(guarded(motion_call(step)))
    lines += ["}", ""]
    return "\n".join(lines)
