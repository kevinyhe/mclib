"""A routine as an mclib autonomous() in C++.

Paths use the code in path_helpers.hpp, copied into the output, so the robot
bakes, plans and follows each path exactly as the simulator did."""
from pathlib import Path

from geometry import designed_poses, path_layout
from project import PRESETS, STEP_TYPES, TRACKING, carries_speed, step_time_limit

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


def exported_helpers():
    text = (HERE / "path_helpers.hpp").read_text()
    begin, end = "// BEGIN EXPORTED HELPERS\n", "// END EXPORTED HELPERS"
    return text[text.index(begin) + len(begin):text.index(end)].rstrip() + "\n"


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
    lines += ["using namespace mclib::units::literals;", "using mclib::control::MotionResult;", "",
              "void autonomous() {",
              "  mclib::control::DriveHardware* drive = mclib::control::requireDrive(\"autonomous\");",
              "  if (drive == nullptr) return;",
              f"  drive->setHeadingDeg({number(start['heading'])});",
              f"  mclib::control::resetOdometry({{{number(start['x'])}, {number(start['y'])}, "
              f"{number(start['heading'])} * mclib::kPi / 180}});",
              f"  mclib::control::robotState().setCorrectAngleDeg({number(start['heading'])});"]
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
            lines += [f"    }}, limits{join});",
                      "  " + guarded(f"path_planner::followSections(plan, follow, {millis(timeout)}, "
                                     f"{volts(step['volts'])}, {exit_flag(step)})"),
                      "  }"]
        elif step["type"] == "wall_reset":
            call = (f"wallReset({inches(step['x'])}, {inches(step['y'])}, {degrees(step['heading'])}, "
                    f"{volts(step['volts'])}, {millis(step['timeout_ms'])}, "
                    f"{number(step['current_ma'])} * mclib::units::milliampere)")
            lines.append(f"  if (!{call}) return;" if stop else f"  {call};")
        else:
            lines.append(guarded(motion_call(step)))
    lines += ["}", ""]
    return "\n".join(lines)
