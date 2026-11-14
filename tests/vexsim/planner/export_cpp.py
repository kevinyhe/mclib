"""A routine as the files that put it on a robot running mclib.

The routine is one function in its own .cpp. Paths use the code in
path_helpers.hpp and the lift and claw use cascade_helpers.hpp, both copied
out as headers, so the robot bakes, plans and follows each path exactly as
the simulator did. The robot's own numbers and ports go in headers that are
written once and then left alone."""
from pathlib import Path
import re

from mechanisms import ACTION_TIMES, MECHANISM_ENUMS, MECHANISMS, PRELOADS, cpp_preset, describe_match_load
from geometry import designed_poses, path_layout
from project import MECHANISM_STEPS, PRESETS, STEP_TYPES, TRACKING, TRACTION, carries_speed, step_time_limit

HERE = Path(__file__).resolve().parent
PLACEHOLDER_FEEDFORWARD = dict(ks=0.7, kv=0.13, top_ips=60.0, track_in=13.5)
ROUTINE_FOLDER = "src/autons"
HEADER_FOLDER = "include/planner"
#: Who changes a file after its first export: "planner" files are replaced by
#: every export, "you" files are written once and never replaced.
PLANNER, YOU = "planner", "you"


def routine_words(name):
    return re.findall(r"[a-z0-9]+", name.lower())


def function_name(name):
    return "run" + ("".join(word.capitalize() for word in routine_words(name)) or "Routine")


def file_stem(name):
    return "_".join(routine_words(name)) or "routine"


def comment_text(text):
    """@p text safe to end a // comment with: on one line, and not ending in
    a backslash, which would pull the next line of code into the comment."""
    return re.sub(r"[\x00-\x1f\x7f]+", " ", text).rstrip("\\ ")


def string_literal(text):
    return '"' + text.replace("\\", "\\\\").replace('"', '\\"') + '"'


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


def helper_file(name, what):
    return dict(path=f"{HEADER_FOLDER}/{name}", owner=PLANNER,
                about=f"Shared code: {what} Exporting again replaces it.",
                code="\n".join([f"// From mclib tests/vexsim/planner/{name} (Mozilla Public License 2.0).",
                                f"// {what}",
                                "// The planner replaces this file on every export.",
                                "#pragma once", "", "#include \"mclib/mclib.hpp\"", "",
                                exported_helpers(name)]))


def robot_drive_file(project, settings):
    feedforward = settings["feedforward"] if settings else PLACEHOLDER_FEEDFORWARD
    source = (f"were measured on the simulated robot:\n// {PRESETS[project['robot']['preset']]['label']}."
              if settings else "are placeholders.")
    return dict(path=f"{HEADER_FOLDER}/robot_drive.hpp", owner=YOU,
                about="Your robot's kS, kV and track width. Written once, then never replaced: "
                      "put your measured numbers in it.",
                code=f"""// Your robot's drive numbers, for routines exported from the mclib path planner.
// The planner writes this file once and never replaces it, so edit it freely.
#pragma once

#include "mclib/mclib.hpp"

namespace planner_robot {{

// kS, kV and the track width {source}
// Measure your own robot (mclib docs/motion.md) and put its numbers here.
inline mclib::control::RamseteConfig followConfig() {{
  using namespace mclib::units::literals;
  mclib::control::RamseteConfig follow;
  follow.feedforward.kS = {volts(feedforward['ks'])};
  follow.feedforward.kV = {number(feedforward['kv'])}_V / mclib::units::inps;
  follow.track_width = {inches(feedforward['track_in'])};
  return follow;
}}

}}  // namespace planner_robot
""")


def claw_start(project):
    return "closed" if project["preload"] != "none" else "open"


def robot_cascade_file(project):
    start = claw_start(project)
    return dict(path=f"{HEADER_FOLDER}/robot_cascade.hpp", owner=YOU,
                about="Your robot's lift, claw and roller ports. Written once, then never replaced: "
                      "set the ports to your robot's.",
                code=f"""// Your robot's lift, claw and roller, for routines exported from the mclib
// path planner. The planner writes this file once and never replaces it, so
// edit it freely.
#pragma once

#include "mclib/mclib.hpp"
#include "planner/cascade_helpers.hpp"

namespace planner_robot {{

// The ports are placeholders: set them to your robot's. The three lift motors
// share one shaft, so the middle one is reversed. Before the match, with the
// lift all the way down, power on the robot so the encoders read 0. The
// claw's solenoid is on 3-wire port A; true closes it. The claw is {start}
// when the program starts.
inline mclib::device::MotorGroup lift_motors({{7, -8, 9}}, mclib::device::Gearset::Green);
inline mclib::device::Pneumatic claw_piston('A');
inline mclib::device::MotorGroup roller_motor({{10}}, mclib::device::Gearset::Green);
inline cascade::Rig cascade_rig([] {{ return lift_motors.getAveragePositionDeg(); }},
                                [](double volts) {{ lift_motors.setVoltage(volts); }},
                                [](bool closed) {{ claw_piston.set_value(closed); }},
                                [] {{ return roller_motor.getAveragePositionDeg(); }},
                                [](double volts) {{ roller_motor.setVoltage(volts); }},
                                {cpp_preset("claw", start)});

// Runs the lift and the actions of each motion every 10 ms, as the simulator
// did. It keeps holding the last lift preset after the routine ends.
inline void startCascadeTask() {{
  static pros::Task cascade_task([] {{
    while (true) {{
      const auto pose = mclib::control::robotState().pose();
      cascade_rig.tick(pose.x, pose.y, mclib::time::millis());
      pros::delay(10);
    }}
  }});
}}

}}  // namespace planner_robot
""")


def cascade_start(project):
    """The lines that start the lift's task and put the claw where the
    simulated routine started with it."""
    start = claw_start(project)
    lines = []
    if project["preload"] != "none":
        lines.append(f"  // Preload: {PRELOADS[project['preload']]}.")
    lines += ["  cascade::Rig& cascade_rig = planner_robot::cascade_rig;",
              "  planner_robot::startCascadeTask();",
              f"  // The simulated routine started with the claw {start}.",
              f"  if ({'!' if start == 'closed' else ''}cascade_rig.clawClosed()) "
              f"cascade_rig.setClaw({cpp_preset('claw', start)});"]
    return lines


def action_literal(action):
    when = "cascade::Action::When::" + {"start": "Start", "distance": "AfterDistance", "time": "AfterTime",
                                        "end": "End"}[action["when"]]
    preset = f"static_cast<int>({cpp_preset(action['mechanism'], action['preset'])})"
    return f"{{{when}, {number(action['value'])}, {MECHANISM_ENUMS[action['mechanism']]}, {preset}}}"


def with_actions(step, call):
    """A motion call wrapped so the step's mechanism actions run during it."""
    literals = ", ".join(action_literal(action) for action in step["actions"])
    amounts = {"distance": "in from the step start", "time": "ms"}
    when = "; ".join(f"{MECHANISMS[action['mechanism']]['label']} to "
                     f"{MECHANISMS[action['mechanism']]['presets'][action['preset']]} "
                     f"{ACTION_TIMES[action['when']].lower()}"
                     + (f" {number(action['value'])} {amounts[action['when']]}" if action["when"] in amounts else "")
                     for action in step["actions"])
    return ["  {", f"    // During this motion: {when}.",
            "    const auto pose = mclib::control::robotState().pose();",
            f"    cascade_rig.beginActions({{{literals}}}, pose.x, pose.y, mclib::time::millis());",
            f"    {call};",
            "    cascade_rig.finishActions();", "  }"]


def lift_call(step):
    return f"cascade_rig.moveLift({cpp_preset('lift', step['lift_preset'])}, {step['timeout_ms']})"


def claw_call(step):
    return f"cascade_rig.setClaw({cpp_preset('claw', step['claw_preset'])})"


def roller_call(step):
    return (f"cascade_rig.spinRoller(*drive, {number(step['roller_deg'])}, {number(step['roller_volts'])}, "
            f"{number(step['press_volts'])}, {step['timeout_ms']})")


def match_load_comment(step):
    return (f"  // The Drive Team stands {describe_match_load(step)}. The robot does nothing. Only legal in the "
            "Driver Controlled Period (SG11a).")


def describe(step, index):
    label = STEP_TYPES[step["type"]]["label"]
    note = comment_text(step.get("note", ""))
    return f"// {index + 1}. {label}{': ' + note if note else ''}"


def exit_flag(step):
    return "false" if step["chain"] else "true"


def motion_call(step):
    """The mclib call for a drive step. A Min voltage left on automatic is
    left out, so the call uses mclib's default."""
    kind, t, v = step["type"], step["timeout_ms"], step["volts"]
    direction = step.get("direction", 1)
    stop = exit_flag(step)
    floor = "" if step["min_volts"] is None else f", {volts(step['min_volts'])}"
    overturned_floor = "" if step["min_volts"] is None else f", true{floor}"
    if kind == "drive":
        return f"driveTo({inches(step['distance'])}, {millis(t)}, {stop}, {volts(v)}{floor})"
    if kind == "turn":
        return f"turnToAngle({degrees(step['heading'])}, {millis(t)}, {stop}, {volts(v)}{floor})"
    if kind == "point":
        return (f"moveToPoint({inches(step['x'])}, {inches(step['y'])}, {direction}, {millis(t)}, {stop}, "
                f"{volts(v)}{overturned_floor})")
    if kind == "turn_to_point":
        return f"turnToPoint({inches(step['x'])}, {inches(step['y'])}, {direction}, {millis(t)}{floor})"
    if kind == "boomerang":
        return (f"boomerang({inches(step['x'])}, {inches(step['y'])}, {direction}, {degrees(step['heading'])}, "
                f"{number(step['lead'])}, {millis(t)}, {stop}, {volts(v)}{overturned_floor})")
    if kind == "arc":
        return (f"curveCircle({degrees(step['heading'])}, {inches(step['radius'])}, {millis(t)}, {stop}, "
                f"{volts(v)}{floor})")
    if kind == "reverse_arc":
        return (f"curveCircleReverse({degrees(step['heading'])}, {inches(step['radius'])}, {millis(t)}, "
                f"{stop}, {volts(v)}{floor})")
    if kind == "swing":
        return f"swing({degrees(step['heading'])}, {direction}, {millis(t)}, {stop}, {volts(v)}{floor})"
    raise ValueError(f"no motion call for {kind}")


def hookup_lines(project):
    """The declaration and the selector line that make the routine runnable."""
    function = function_name(project["name"])
    return f"void {function}();", f"selector.add({string_literal(project['name'])}, {function});"


def hookup(project):
    """What to add to the robot's main.cpp so the selector can run the routine."""
    declaration, registration = hookup_lines(project)
    return "\n".join(["// Add these two lines to src/main.cpp.",
                      "// `selector` is your mclib::auton::AutonSelector.",
                      "", "// Above initialize():", declaration,
                      "", "// In initialize(), with your other routines:", registration, ""])


def path_lines(project, index, step, start_pose, plan, top_ips):
    defaults = project["path_defaults"]
    if not step["points"]:
        return ["  // This path has no points yet."]
    lines = []
    layout = path_layout(step, start_pose)
    timeout = step_time_limit(step, plan) if plan or step["timeout_ms"] is not None else 8000
    if plan:
        lines.append(f"  // Planned in the simulator: {plan['duration_s']:.2f} s, {plan['length_in']:.1f} in, "
                     f"top speed {plan['top_speed_ips']:.1f} in/s.")
    lines += ["  {", "    const auto plan = path_planner::planSections({"]
    for section in layout["sections"]:
        percent = section["speed_percent"] if section["speed_percent"] is not None else defaults["speed_percent"]
        points = ", ".join(f"{{{number(p[0])}, {number(p[1])}}}" for p in
                           (section["start"], section["start_handle"], section["end_handle"], section["end"]))
        lines.append(f"        {{{points}, {number(percent / 100 * top_ips)}, "
                     f"{'true' if section['reverse'] else 'false'}}},")
    if carries_speed(project, index) and not plan:
        lines.append("  // The speed at the join into the next path is 0 until the planner has measured "
                     "the robot. Export again once it has.")
    join = (f", {number(plan['requested_start_ips'])}, {number(plan['requested_end_ips'])}"
            if plan and (plan["requested_start_ips"] or plan["requested_end_ips"]) else "")
    follow_call = (f"path_planner::followSections(plan, follow, {millis(timeout)}, "
                   f"{volts(step['volts'])}, {exit_flag(step)})")
    lines.append(f"    }}, limits{join});")
    if step["actions"]:
        lines += ["  " + line for line in with_actions(step, follow_call)]
    else:
        lines.append(f"    {follow_call};")
    lines.append("  }")
    return lines


def step_lines(project, index, step, start_pose, plan, top_ips):
    kind = step["type"]
    if kind == "path":
        return path_lines(project, index, step, start_pose, plan, top_ips)
    if kind == "lift":
        return [f"  {lift_call(step)};"]
    if kind == "claw":
        return [f"  {claw_call(step)};"]
    if kind == "roller":
        return [f"  {roller_call(step)};"]
    if kind == "match_load":
        return [match_load_comment(step)]
    if kind == "wait":
        return [f"  pros::delay({step['wait_ms']});"]
    if kind == "wall_reset":
        call = (f"wallReset({inches(step['x'])}, {inches(step['y'])}, {degrees(step['heading'])}, "
                f"{volts(step['volts'])}, {millis(step['timeout_ms'])}, "
                f"{number(step['current_ma'])} * mclib::units::milliampere)")
    else:
        call = motion_call(step)
    return with_actions(step, call) if step["actions"] else [f"  {call};"]


def simulated_on(project):
    """The comment lines that say what the routine was simulated with."""
    robot = project["robot"]
    wheels = "" if robot["traction"] == "none" else f"; traction wheels: {TRACTION[robot['traction']].lower()}"
    lines = [f"// Simulated on: {PRESETS[robot['preset']]['label']}; {TRACKING[robot['tracking']].lower()}{wheels}."]
    others = [f"the {other['role']} robot " + (f"driving a {len(other['path'])}-point path" if other["path"]
                                                else "holding still") for other in project["other_robots"]]
    if others:
        lines.append(f"// Simulated with {', '.join(others)}. Real ones will be placed and driven differently.")
    return lines


def routine_code(project, settings, plans):
    top_ips = (settings["feedforward"] if settings else PLACEHOLDER_FEEDFORWARD)["top_ips"]
    defaults, robot, start = project["path_defaults"], project["robot"], project["start"]
    poses = designed_poses(project)
    has_path = any(step["type"] == "path" for step in project["steps"])
    cascade = uses_cascade(project)
    function = function_name(project["name"])
    includes = ["mclib/mclib.hpp"]
    if cascade:
        includes += ["planner/cascade_helpers.hpp", "planner/robot_cascade.hpp"]
    if has_path:
        includes += ["planner/path_helpers.hpp", "planner/robot_drive.hpp"]
    lines = [f"// \"{comment_text(project['name'])}\", exported from the mclib path planner.",
             "// The planner replaces this file on every export.",
             *simulated_on(project),
             "// Units: inches and degrees. 0 deg faces +Y (forward); clockwise is positive.",
             f"// Place the robot at ({number(start['x'])}, {number(start['y'])}) facing "
             f"{number(start['heading'])} deg before autonomous starts.",
             "//",
             "// To run it from your selector, add to src/main.cpp:",
             *(f"//   {line}" for line in hookup_lines(project)),
             *(f"#include \"{include}\"" for include in sorted(includes)), "",
             "using namespace mclib::units::literals;", "",
             f"void {function}() {{",
             f"  mclib::control::DriveHardware* drive = mclib::control::requireDrive(\"{function}\");",
             "  if (drive == nullptr) return;",
             f"  drive->setHeadingDeg({number(start['heading'])});",
             f"  mclib::control::resetOdometry({{{number(start['x'])}, {number(start['y'])}, "
             f"{number(start['heading'])} * mclib::kPi / 180}});",
             f"  mclib::control::robotState().setCorrectAngleDeg({number(start['heading'])});"]
    speed_up, slow_down = number(project["slew"]["speed_up_v"]), number(project["slew"]["slow_down_v"])
    lines += ["  // How fast drive steps may raise and lower the drive voltage, volts per 10 ms.",
              "  mclib::control::MotionConfig motion = mclib::control::motionConfig();",
              f"  motion.slew = {{{speed_up}, {slow_down}, {speed_up}, {slow_down}}};",
              "  mclib::control::setMotionConfig(motion);"]
    if cascade:
        lines += cascade_start(project)
    if has_path:
        lines += ["", "  // kS, kV and the track width are your robot's, from planner/robot_drive.hpp.",
                  "  mclib::control::RamseteConfig follow = planner_robot::followConfig();",
                  f"  follow.gains.b = {number(defaults['ramsete_b'])};",
                  f"  follow.gains.zeta = {number(defaults['ramsete_zeta'])};",
                  f"  const path_planner::Limits limits{{{number(defaults['accel_ips2'])}, "
                  f"{number(defaults['cornering_ips2'])}, follow.track_width.in()}};"]
    for index, step in enumerate(project["steps"]):
        lines += ["", "  " + describe(step, index)]
        if step["chain"]:
            lines.append("  // Keeps moving into the next step: exit = false, so it does not stop or settle.")
        lines += step_lines(project, index, step, poses[index][0], plans[index] if plans else None, top_ips)
    lines += ["}", ""]
    return "\n".join(lines)


def export(project, settings=None, plans=None):
    """The files that put @p project on a robot, the routine first. @p settings
    and @p plans come from a plan; without them the drive numbers are
    placeholders to measure on the robot. `drive_numbers` says which, and is
    None when no step needs them."""
    files = [dict(path=f"{ROUTINE_FOLDER}/{file_stem(project['name'])}.cpp", owner=PLANNER,
                  about="The routine. Exporting again replaces it.",
                  code=routine_code(project, settings, plans))]
    has_path = any(step["type"] == "path" for step in project["steps"])
    cascade = uses_cascade(project)
    if has_path:
        files.append(robot_drive_file(project, settings))
    if cascade:
        files.append(robot_cascade_file(project))
    if has_path:
        files.append(helper_file("path_helpers.hpp",
                                 "bakes each path, plans its speeds and follows it, as the simulator did."))
    if cascade:
        files.append(helper_file("cascade_helpers.hpp",
                                 "the lift presets, the claw and the actions run during a motion, as the "
                                 "simulator ran them."))
    drive_numbers = ("measured" if settings else "placeholders") if has_path else None
    return dict(function=function_name(project["name"]), files=files, hookup=hookup(project),
                drive_numbers=drive_numbers)
