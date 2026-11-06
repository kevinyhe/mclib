"""The path planner's project file: what it holds, its defaults, and the checks
that turn a bad file into a message naming the step and the field."""
import copy
import json
import math

from fields import GAME_FIELDS, NEW_ROUTINE_FIELD, OLD_FILE_FIELD
from mechanisms import (ACTION_TIMES, ACTION_UNITS, CLAW_PRESETS, LIFT_PRESETS, LOADERS,
                        MATCH_LOAD_PINS, MECHANISMS, PRELOADS, RETIRED_LIFT_PRESETS, RETIRED_MECHANISMS,
                        RETIRED_PRELOADS)

FORMAT = "mclib-path-planner"
VERSION = 1
#: The min_output global a motion's min_speed of -1 V stands for
MCLIB_MIN_VOLTS = 1.5
#: Fields older files may have that the planner no longer uses: every run
#: now goes on to the next step unless a step breaks a rule
RETIRED_PROJECT_FIELDS = {"stop_on_failure"}

PRESETS = {
    "six_motor_450": dict(label="6 motors, 450 RPM, 3.25 in wheels", track_width_in=11.5,
                          body_length_in=15.0, body_width_in=12.5, drive_watts=66),
    "four_motor_450": dict(label="4 motors, 450 RPM, 3.25 in wheels", track_width_in=11.5,
                           body_length_in=15.0, body_width_in=12.5, drive_watts=44),
    "four_motor_200": dict(label="4 motors, 200 RPM, 3.25 in wheels", track_width_in=11.5,
                           body_length_in=15.0, body_width_in=12.5, drive_watts=44),
    "speed_base": dict(label="4 motors, 450 RPM, 4 in wheels", track_width_in=11.0,
                       body_length_in=15.0, body_width_in=12.5, drive_watts=44),
    "override_cascade": dict(label="Override cascade robot: 4 motors, 450 RPM, 2.75 in wheels, 14 lb",
                             track_width_in=11.5, body_length_in=15.0, body_width_in=12.5, drive_watts=44),
}
#: The Override robot's drive: four 11 W motors, 44 W. The lift's three 11 W motors make 77 W of the 88 W; the
#: claw is pneumatic
OVERRIDE_PRESET = "override_cascade"
OVERRIDE_TRACKING = "horizontal"
TRACKING = {
    "two": "Two tracking wheels (forward and sideways)",
    "horizontal": "One sideways tracking wheel, drive encoders forward",
    "drive": "Drive motor encoders only",
}
HEADING_SOURCES = {
    "imu": "Inertial sensor (IMU)",
    "encoders": "Drive encoders (no IMU)",
}

GAINS = {f"{loop}_{term}": dict(default=value, label=f"{name} {term[1:].upper()}")
         for loop, name, values in (("drive", "Drive", (.4, 0, 3)), ("heading", "Heading", (.3, 0, 1.5)),
                                    ("turn", "Turn", (.3, 0, 1.5)))
         for term, value in zip(("kp", "ki", "kd"), values)}

FIELDS = {
    "x": dict(label="X", unit="in", low=-500, high=500, step=.5,
              help="Inches right of the field centre. Negative is left."),
    "y": dict(label="Y", unit="in", low=-500, high=500, step=.5,
              help="Inches forward of the field centre. Negative is toward the driver."),
    "heading": dict(label="Heading", unit="°", low=-3600, high=3600, step=1,
                    help="Which way the robot faces. 0° faces +Y (forward). Clockwise is positive."),
    "distance": dict(label="Distance", unit="in", low=-500, high=500, step=1,
                     help="How far to drive along the current heading. Negative drives backward."),
    "radius": dict(label="Radius", unit="in", low=-250, high=250, step=.5,
                   help="Radius of the arc. Positive curves right, negative curves left. At least 1 in."),
    "direction": dict(label="Direction", choices={"1": "Forward", "-1": "Backward"},
                      help="Drive forward, or back the robot in."),
    "lead": dict(label="Lead", unit="", low=0, high=.99, step=.01,
                 help="How far behind the target the robot aims, as a fraction of the distance left. "
                      "Larger swings wider to arrive straighter."),
    "current_ma": dict(label="Stall current", unit="mA", low=0, high=10000, step=50,
                       help="Motor current that counts as pushing against the wall."),
    "wait_ms": dict(label="Wait", unit="ms", low=10, high=120000, step=10,
                    help="How long to wait. The step always takes exactly this long."),
    "timeout_ms": dict(label="Time limit", unit="ms", low=50, high=20000, step=50,
                       help="The motion gives up here and the next step starts."),
    "volts": dict(label="Max voltage", unit="V", low=.1, high=12, step=.1,
                  help="Caps the drive voltage. 12 V is full power."),
    "min_volts": dict(label="Min voltage", unit="V", low=0, high=12, step=.1, automatic=MCLIB_MIN_VOLTS,
                      help_by_step={
                          **dict.fromkeys(("drive", "point", "boomerang", "arc", "reverse_arc"),
                                          "The least voltage the drive gets until the motion ends, so it does not "
                                          "stall short of the target. Automatic is mclib's 1.5 V, and only while "
                                          "the step keeps moving into the next one. 0 turns it off. On a step "
                                          "that stops, too high a floor keeps the robot rocking across the "
                                          "target until the time limit."),
                          **dict.fromkeys(("turn", "swing"),
                                          "The least voltage while this turn keeps moving into the next step. A "
                                          "turn that stops at the end has no floor. Automatic is mclib's 1.5 V."),
                          "turn_to_point": "The least voltage turnToPoint() uses for the whole turn. Automatic is "
                                           "mclib's 1.5 V. 0 turns it off."}),
    "lift_preset": dict(label="Lift position", choices=LIFT_PRESETS,
                        help="How high the middle of the claw is above the floor. Each jaw holds a stack's Cup "
                             "below its rim and its Pin above its flange. Over carries the Cup 0.25 in above the "
                             "tip of the Pin in that Goal; On sets the Cup's rim on that Pin."),
    "claw_preset": dict(label="Claw", choices=CLAW_PRESETS,
                        help="A pneumatic piston closes both jaws in 130 ms and opens them in 160 ms, as fast as "
                             "air gets through the valve. Each V-shaped jaw holds a stack at two heights: the Cup "
                             "below its rim and the Pin above its flange. It also holds a lone Cup, and a Pin hung "
                             "by its flange. A Pin standing alone on the floor is too thin where the jaws reach."),
    "roller_deg": dict(label="Roller turn", unit="°", low=10, high=3600, step=10,
                       help="How far the roller turns, read from its motor's encoder. Each flap turns a Toggle one "
                            "face. Starting with the flaps level and the back 1 in off the wall, at 12 V a flap "
                            "reaches the bar at about 177° and seats the first face by 255°; the next flap reaches "
                            "it at 357° and seats the second face by 435°, and a third would reach it at 530°."),
    "roller_volts": dict(label="Roller voltage", unit="V", low=1, high=12, step=.5,
                         help="The roller turns the flaps behind its axle up, which turns a Toggle's top toward "
                              "the wall. Turned the other way the flaps press the Toggle onto its plates and it "
                              "does not move, so the step only turns it this way."),
    "press_volts": dict(label="Drive pushing back", unit="V", low=0, high=12, step=.5,
                        help="The drive pushes the robot back onto the wall while the roller turns. Each flap that "
                             "turns a Toggle pushes the robot off the wall. At 0 V a robot 1 in off the wall stays "
                             "there and the flaps miss the bar. From 2 V up both faces seat. The roller tips the "
                             "robot onto its front wheels and the back wheels slip, so odometry ends 0.1 in off at "
                             "2 V, 0.3 in at 3 V and 1.4 in at 4 V; at 4 V the drive pushes with 97% of the grip."),
    "loader": dict(label="Loader", choices=LOADERS,
                   help="The red Loader the Drive Team loads. Left and right are as seen from the red Alliance "
                        "Station."),
    "match_load_pin": dict(label="Pin", choices=MATCH_LOAD_PINS,
                           help="The Pin the Drive Team stands in the Cup. Red has 10 red/yellow Pins and 1 "
                                "yellow/yellow Pin to load in a Match."),
}

STEP_TYPES = {
    "path": dict(label="Path", call="followTrajectory()",
                 description="Follow a curve through the points. mclib plans the speeds, then RAMSETE "
                             "steers the robot along the plan from odometry.",
                 fields=[], defaults=dict(points=[], sections=[], start_handle=None, timeout_ms=None, volts=12)),
    "drive": dict(label="Drive straight", call="driveTo()",
                  description="Drive a distance along the current heading.",
                  fields=["distance"], defaults=dict(distance=24, min_volts=None)),
    "turn": dict(label="Turn to heading", call="turnToAngle()",
                 description="Turn in place to face a field heading.",
                 fields=["heading"], defaults=dict(heading=90, min_volts=None)),
    "turn_to_point": dict(label="Turn to face a point", call="turnToPoint()",
                          description="Turn in place to face a field position. Backward points the back of the robot at it.",
                          fields=["x", "y", "direction"], defaults=dict(x=24, y=24, direction=1, min_volts=None)),
    "point": dict(label="Drive to a point", call="moveToPoint()",
                  description="Turn toward a field position and drive to it.",
                  fields=["x", "y", "direction"], defaults=dict(x=24, y=24, direction=1, min_volts=None)),
    "boomerang": dict(label="Drive to a pose", call="boomerang()",
                      description="Drive to a field position and arrive facing a heading, aiming at a moving point "
                                  "behind the target.",
                      fields=["x", "y", "heading", "direction", "lead"],
                      defaults=dict(x=24, y=24, heading=90, direction=1, lead=.5, min_volts=None)),
    "arc": dict(label="Arc", call="curveCircle()",
                description="Drive around a circle of the given radius until facing the heading.",
                fields=["heading", "radius"], defaults=dict(heading=90, radius=24, min_volts=None)),
    "reverse_arc": dict(label="Reverse arc", call="curveCircleReverse()",
                        description="Back around a circle of the given radius until facing the heading. "
                                    "The radius sign and the heading change must describe backing up.",
                        fields=["heading", "radius"], defaults=dict(heading=-90, radius=24, min_volts=None)),
    "swing": dict(label="Swing turn", call="swing()",
                  description="Turn by driving one side while the other side holds still.",
                  fields=["heading", "direction"], defaults=dict(heading=90, direction=1, min_volts=None)),
    "wall_reset": dict(label="Wall reset", call="wallReset()",
                       description="Push into a wall until the motors stall, then set odometry to this pose. "
                                   "On a game field the robot pushes against the real wall. On the open floor "
                                   "there are no walls, so the drive is held still as if it were touching one. "
                                   "Must be the last step, because it moves the odometry frame.",
                       fields=["x", "y", "heading", "current_ma"],
                       defaults=dict(x=0, y=0, heading=0, current_ma=2500)),
    "lift": dict(label="Move the lift", call="moveLift()", mechanism="lift", still=True,
                 description="Raise or lower the cascade lift to a preset and wait until it gets there. The drive "
                             "holds still. The lift is an mclib PresetPositionMechanism on three motors.",
                 fields=["lift_preset"], defaults=dict(lift_preset="over_neutral_goal", timeout_ms=1500)),
    "claw": dict(label="Open or close the claw", call="setClaw()", mechanism="claw", still=True, fixed_ms=170,
                 description="Open or close the pneumatic claw. The piston takes 130 ms to close and 160 ms "
                             "to open; the step waits 170 ms. "
                             "To pick up a stack, open the claw with the lift Down and drive the bite onto a Cup "
                             "with a Pin in it until the Cup stops against the plate behind the jaws, then close.",
                 fields=["claw_preset"], defaults=dict(claw_preset="closed")),
    "roller": dict(label="Turn a Toggle with the roller", call="spinRoller()", mechanism="roller", still=True,
                   description="Back onto the wall at a Toggle first, its flaps 1 in or less from the bar. The "
                               "roller turns as far as you set while the drive pushes the robot back onto the "
                               "wall, then stops. Each flap turns the Toggle one face, its top toward the wall. "
                               "On red's wall one face from yellow is blue and two are red; on blue's wall, red "
                               "then blue.",
                   fields=["roller_deg", "roller_volts", "press_volts"],
                   defaults=dict(roller_deg=470, roller_volts=12, press_volts=3, timeout_ms=2000)),
    "wait": dict(label="Wait", call="pros::delay()", still=True, timed_by="wait_ms",
                 description="Wait with the drive as the last step left it, then go on. Use it to wait for the "
                             "Driver Controlled Period, 15 s into the match, before a Match Load.",
                 fields=["wait_ms"], defaults=dict(wait_ms=1000)),
    "match_load": dict(label="Match load", call="Drive Team", still=True, fixed_ms=0,
                       description="The Drive Team lifts a red Loader by its handle and stands a Cup with a Pin in "
                                   "it on the floor under it, through the Loader's back. The Loader stays up. The "
                                   "robot does nothing in this step. Match Loads may only be loaded in the Driver "
                                   "Controlled Period (SG11a): in the first 15 s this step breaks that rule and "
                                   "the run stops after it. In Skills they may be loaded at any time (RSC2d), "
                                   "which the planner does not model.",
                       fields=["loader", "match_load_pin"],
                       defaults=dict(loader="red_left", match_load_pin="red_yellow_up")),
}
MECHANISM_STEPS = {kind for kind, info in STEP_TYPES.items() if "mechanism" in info}
STILL_STEPS = {kind for kind, info in STEP_TYPES.items() if info.get("still")}

ACTIONS = dict(
    label="Mechanisms during this step",
    help="Moves the lift or the claw while the robot drives. Anything not run by the end of the motion runs then.",
    times=ACTION_TIMES, units=ACTION_UNITS,
    mechanisms={key: dict(label=item["label"], presets=item["presets"]) for key, item in MECHANISMS.items()},
    max=6, max_distance_in=500, max_time_ms=20000,
)
PRELOAD = dict(label="Preload", choices=PRELOADS,
               help="The red/yellow Pin each robot must touch at the start (SG5). It hangs in the closed claw "
                    "by its flange with the half you choose up, and the claw holds it until a step opens the "
                    "claw.")

CHAIN = dict(
    label="Keep moving into the next step",
    help="Runs this step with exit = false. It ends as soon as the robot reaches or passes the target, without "
         "stopping or settling, and the next step starts at once with the drive still powered. A path that keeps "
         "moving into another path carries its speed across the join. Faster than stopping, but less exact.",
    not_allowed={
        "turn_to_point": "turnToPoint() has no exit option in mclib, so it always stops.",
        "wall_reset": "A wall reset has to stop against the wall.",
        "lift": "The drive is not moving during a lift step.",
        "claw": "The drive is not moving during a claw step.",
        "roller": "The drive pushes the robot onto the wall during a roller step.",
        "match_load": "The drive is not moving during a Match Load.",
        "wait": "The drive is not moving during a wait.",
    },
    last_step="The last step has to stop: there is no next step to keep moving into.",
)

PATH_DEFAULTS = {
    "speed_percent": dict(default=70, label="Speed limit", unit="% of top speed", low=5, high=100, step=1,
                          help="Default speed limit for each section of a path. Top speed is measured in the simulator."),
    "accel_ips2": dict(default=60, label="Acceleration", unit="in/s²", low=5, high=400, step=5,
                       help="How fast the planned speed may rise or fall."),
    "cornering_ips2": dict(default=60, label="Cornering", unit="in/s²", low=5, high=400, step=5,
                           help="Sideways acceleration allowed on curves. Lower slows down for turns."),
    "ramsete_b": dict(default=50, label="RAMSETE b", unit="", low=.1, high=500, step=1,
                      help="How hard RAMSETE pulls back onto the path. 50 suits these simulated robots."),
    "ramsete_zeta": dict(default=.7, label="RAMSETE zeta", unit="", low=0, high=1, step=.05,
                         help="Damping. Larger overshoots less."),
}
SLEW = {
    "speed_up_v": dict(default=1, label="Speed-up limit", unit="V per 10 ms", low=.05, high=12, step=.05,
                       help="How fast drive, turn, point and arc steps may raise the drive voltage. mclib's default "
                            "is 1 V: 0 to 12 V in 0.12 s. Lower keeps the wheels from spinning as the robot starts, "
                            "which drive encoders count as distance."),
    "slow_down_v": dict(default=1, label="Slow-down limit", unit="V per 10 ms", low=.05, high=12, step=.05,
                        help="How fast those steps may lower the drive voltage. mclib's default is 1 V."),
}
CONDITIONS = {
    "battery_percent": dict(default=100, label="Battery charge", unit="%", low=0, high=100, step=5,
                            help="Lower charge gives less voltage under load."),
    "grip_percent": dict(default=100, label="Floor grip", unit="% of normal", low=5, high=200, step=5,
                         help="Lower grip makes the wheels slip more."),
    "noise_seed": dict(default=1, label="Sensor noise seed", unit="", low=0, high=2147483647, step=1,
                       help="Change it to get different IMU noise. The same seed repeats a run exactly."),
}

LIMITS = dict(max_steps=40, max_points=30, max_simulated_s=120, max_body_bytes=262144,
              queue_size=4, history_size=40, hold_ms=300, min_point_gap_in=.5, min_handle_in=.5,
              max_handle_in=300, settle_allowance_ms=4000, chain_allowance_ms=500)

DEFAULT_PROJECT = dict(
    format=FORMAT, version=VERSION, name="New routine", field=OLD_FILE_FIELD, preload="none",
    robot=dict(preset="six_motor_450", tracking="two", heading_source="imu"),
    conditions={key: item["default"] for key, item in CONDITIONS.items()},
    gains={key: item["default"] for key, item in GAINS.items()},
    slew={key: item["default"] for key, item in SLEW.items()},
    path_defaults={key: item["default"] for key, item in PATH_DEFAULTS.items()},
    start=dict(x=0, y=-48, heading=0),
    steps=[dict(type="path", chain=False, **copy.deepcopy(STEP_TYPES["path"]["defaults"]))],
)


class ProjectError(ValueError):
    """A problem in the project, with the index of the step it is in."""

    def __init__(self, message, step=None):
        super().__init__(message)
        self.step = step


def strict_json(text):
    """json.loads that also rejects duplicate keys and NaN/Infinity."""
    def pairs(items):
        result = {}
        for key, value in items:
            if key in result:
                raise ValueError(f"The file repeats the key \"{key}\"")
            result[key] = value
        return result

    def invalid(value):
        raise ValueError(f"The file contains {value}, which is not a number")
    return json.loads(text, parse_constant=invalid, object_pairs_hook=pairs)


def step_name(index, step):
    label = STEP_TYPES.get(step.get("type"), {}).get("label", "Step") if isinstance(step, dict) else "Step"
    return f"Step {index + 1} ({label})"


def _object(value, allowed, where):
    if not isinstance(value, dict):
        raise ValueError(f"{where} must be an object")
    unknown = sorted(set(value) - set(allowed))
    if unknown:
        raise ValueError(f"{where} has fields the planner does not know: {', '.join(unknown)}")
    return value


def _number(value, low, high, where, unit="", integer=False):
    if isinstance(value, bool) or not isinstance(value, (int, float)) or not math.isfinite(value):
        raise ValueError(f"{where} must be a number")
    if not low <= value <= high:
        suffix = f" {unit}" if unit else ""
        raise ValueError(f"{where} must be between {low:g} and {high:g}{suffix}, not {value:g}")
    if integer and int(value) != value:
        raise ValueError(f"{where} must be a whole number")
    return int(value) if integer else value


def _field(step, key, where):
    info = FIELDS[key]
    value = step[key]
    if key == "direction":
        if value not in (1, -1) or isinstance(value, bool):
            raise ValueError(f"{where}: Direction must be 1 (forward) or -1 (backward)")
        return value
    if "choices" in info:
        if value not in info["choices"]:
            raise ValueError(f"{where}: {info['label']} must be one of: {', '.join(info['choices'])}")
        return value
    _number(value, info["low"], info["high"], f"{where}: {info['label']}", info["unit"],
            integer=key in ("timeout_ms", "wait_ms"))
    if key == "radius" and abs(value) < 1:
        raise ValueError(f"{where}: Radius must be at least 1 in either way, not {value:g}")
    if key == "lead" and value >= 1:
        raise ValueError(f"{where}: Lead must be less than 1")
    return value


def _validate_path(raw, step, where):
    points = raw.get("points", [])
    if not isinstance(points, list) or len(points) > LIMITS["max_points"]:
        raise ValueError(f"{where}: a path holds up to {LIMITS['max_points']} points")
    sections = raw.get("sections", [])
    if not isinstance(sections, list) or len(sections) != len(points):
        raise ValueError(f"{where}: needs one section per point ({len(points)}), found "
                         f"{len(sections) if isinstance(sections, list) else 'none'}")
    step["points"], step["sections"] = [], []
    for number, point in enumerate(points, start=1):
        at = f"{where}, point {number}"
        _object(point, {"x", "y", "heading", "handle_in", "handle_out"}, at)
        clean = dict(x=None, y=None, heading=None, handle_in=None, handle_out=None)
        clean.update(point)
        for key in ("x", "y"):
            if clean[key] is None:
                raise ValueError(f"{at}: {FIELDS[key]['label']} is required")
            _number(clean[key], -500, 500, f"{at}: {FIELDS[key]['label']}", "in")
        if clean["heading"] is not None:
            _number(clean["heading"], -3600, 3600, f"{at}: Heading", "°")
        for key, label in (("handle_in", "Curve handle before"), ("handle_out", "Curve handle after")):
            if clean[key] is not None:
                _number(clean[key], LIMITS["min_handle_in"], LIMITS["max_handle_in"], f"{at}: {label}", "in")
        step["points"].append(clean)
    for number, section in enumerate(sections, start=1):
        at = f"{where}, section {number}"
        _object(section, {"speed_percent", "reverse"}, at)
        clean = dict(speed_percent=None, reverse=False)
        clean.update(section)
        if clean["speed_percent"] is not None:
            _number(clean["speed_percent"], 5, 100, f"{at}: Speed limit", "%")
        if not isinstance(clean["reverse"], bool):
            raise ValueError(f"{at}: Reverse must be true or false")
        step["sections"].append(clean)
    if step.get("start_handle") is not None:
        _number(step["start_handle"], LIMITS["min_handle_in"], LIMITS["max_handle_in"],
                f"{where}: Curve handle at the start", "in")
    if step.get("timeout_ms") is not None:
        _number(step["timeout_ms"], 50, 60000, f"{where}: Time limit", "ms", integer=True)


def validate_project(raw):
    """A complete, checked copy of @p raw with every default filled in.

    Raises ValueError with a message that names the step and field."""
    _object(raw, set(DEFAULT_PROJECT) | RETIRED_PROJECT_FIELDS, "The project")
    if raw.get("format", FORMAT) != FORMAT:
        raise ValueError(f"This is not a path planner project (format is \"{raw.get('format')}\")")
    if raw.get("version", VERSION) != VERSION:
        raise ValueError(f"This project is version {raw.get('version')}; this planner reads version {VERSION}")
    project = copy.deepcopy(DEFAULT_PROJECT)
    if "name" in raw:
        project["name"] = raw["name"]
    if not isinstance(project["name"], str) or not 1 <= len(project["name"]) <= 80 or \
            any(ord(c) < 32 for c in project["name"]):
        raise ValueError("The routine name must be 1 to 80 characters on one line")
    project["field"] = raw.get("field", OLD_FILE_FIELD)
    if project["field"] not in GAME_FIELDS:
        raise ValueError(f"Field must be one of: {', '.join(GAME_FIELDS)}")
    project["preload"] = raw.get("preload", "none")
    if project["preload"] in RETIRED_PRELOADS:
        raise ValueError(f"The preload \"{project['preload']}\" was a Pin lying on the floor against the robot. "
                         "The claw now holds a Pin by its flange, so the preload starts in the closed claw. Set "
                         "preload to \"yellow\" or \"red\", the half that points up, or \"none\"")
    if project["preload"] not in PRELOADS:
        raise ValueError(f"Preload must be one of: {', '.join(PRELOADS)}")
    if project["preload"] != "none" and project["field"] != "override":
        raise ValueError("A preload needs the Override field: only the Override robot has a claw")
    project["robot"].update(_object(raw.get("robot", {}), project["robot"], "Robot"))
    for key, choices in (("preset", PRESETS), ("tracking", TRACKING), ("heading_source", HEADING_SOURCES)):
        if project["robot"][key] not in choices:
            raise ValueError(f"Robot {key.replace('_', ' ')} must be one of: {', '.join(choices)}")
    groups = (("conditions", CONDITIONS, "Conditions"), ("gains", GAINS, "Controller gains"),
              ("slew", SLEW, "Speed-up and slow-down"), ("path_defaults", PATH_DEFAULTS, "Path settings"))
    for group, schema, title in groups:
        project[group].update(_object(raw.get(group, {}), schema, title))
        for key, value in project[group].items():
            info = schema[key]
            _number(value, info.get("low", 0), info.get("high", 100), f"{title}: {info['label']}",
                    info.get("unit", ""), integer=key == "noise_seed")
    project["start"].update(_object(raw.get("start", {}), project["start"], "Start pose"))
    for key in ("x", "y", "heading"):
        _number(project["start"][key], FIELDS[key]["low"], FIELDS[key]["high"],
                f"Start pose: {FIELDS[key]['label']}", FIELDS[key]["unit"])
    steps = raw.get("steps", project["steps"])
    if not isinstance(steps, list) or not 1 <= len(steps) <= LIMITS["max_steps"]:
        raise ValueError(f"A routine needs 1 to {LIMITS['max_steps']} steps")
    project["steps"] = []
    for index, value in enumerate(steps):
        try:
            project["steps"].append(_validate_step(index, value, len(steps), project["field"]))
        except ValueError as error:
            raise ProjectError(str(error), index) from error
    return project


def _validate_actions(raw, where):
    if not isinstance(raw, list) or len(raw) > ACTIONS["max"]:
        raise ValueError(f"{where}: up to {ACTIONS['max']} mechanism actions per step")
    actions = []
    for number, action in enumerate(raw, start=1):
        at = f"{where}, action {number}"
        _object(action, {"when", "value", "mechanism", "preset"}, at)
        clean = dict(when="start", value=0, mechanism="lift", preset=None)
        clean.update(action)
        if clean["when"] not in ACTION_TIMES:
            raise ValueError(f"{at}: when must be one of: {', '.join(ACTION_TIMES)}")
        if clean["mechanism"] in RETIRED_MECHANISMS:
            raise ValueError(f"{at}: {RETIRED_MECHANISMS[clean['mechanism']]}")
        if clean["mechanism"] not in MECHANISMS:
            raise ValueError(f"{at}: mechanism must be one of: {', '.join(MECHANISMS)}")
        presets = MECHANISMS[clean["mechanism"]]["presets"]
        if clean["mechanism"] == "lift" and clean["preset"] in RETIRED_LIFT_PRESETS:
            raise ValueError(f"{at}: {retired_lift_preset(clean['preset'])}")
        if clean["preset"] not in presets:
            raise ValueError(f"{at}: the {clean['mechanism']} preset must be one of: {', '.join(presets)}")
        if clean["when"] == "distance":
            _number(clean["value"], 0, ACTIONS["max_distance_in"], f"{at}: distance", "in")
        elif clean["when"] == "time":
            _number(clean["value"], 0, ACTIONS["max_time_ms"], f"{at}: time", "ms", integer=True)
        else:
            clean["value"] = 0
        actions.append(clean)
    return actions


def retired_lift_preset(preset):
    return (f"the lift preset \"{preset}\" held a Pin by its flange, and the claw now holds a Cup by its top rim. "
            f"Choose one of: {', '.join(LIFT_PRESETS)}")


def _validate_step(index, value, count, field):
    where = step_name(index, value)
    if isinstance(value, dict) and value.get("type") in RETIRED_MECHANISMS:
        raise ValueError(f"Step {index + 1}: {RETIRED_MECHANISMS[value['type']]}")
    if isinstance(value, dict) and value.get("type") == "lift" and value.get("lift_preset") in RETIRED_LIFT_PRESETS:
        raise ValueError(f"Step {index + 1} (Move the lift): {retired_lift_preset(value['lift_preset'])}")
    if not isinstance(value, dict) or value.get("type") not in STEP_TYPES:
        raise ValueError(f"Step {index + 1}: unknown step type {value.get('type') if isinstance(value, dict) else value!r}")
    kind = value["type"]
    info = STEP_TYPES[kind]
    _object(value, set(info["defaults"]) | {"type", "note", "timeout_ms", "volts", "chain", "actions"}, where)
    step = dict(type=kind, note="", timeout_ms=4000, volts=6 if kind == "wall_reset" else 12, chain=False,
                actions=[])
    step.update(copy.deepcopy(info["defaults"]))
    step.update(value)
    if not isinstance(step["note"], str) or len(step["note"]) > 80:
        raise ValueError(f"{where}: the note must be text of up to 80 characters")
    if "fixed_ms" in info:
        if value.get("timeout_ms", info["fixed_ms"]) != info["fixed_ms"]:
            raise ValueError(f"{where}: this step always takes {info['fixed_ms']} ms, so it has no time limit. "
                             "Remove timeout_ms")
        step["timeout_ms"] = info["fixed_ms"]
    if "timed_by" in info:
        wait_ms = _field(step, info["timed_by"], where)
        if value.get("timeout_ms", wait_ms) != wait_ms:
            raise ValueError(f"{where}: a wait always takes its {wait_ms:g} ms, so it has no time limit of its "
                             "own. Remove timeout_ms")
        step["timeout_ms"] = wait_ms
    if kind == "path":
        _validate_path(value, step, where)
    else:
        for key in info["fields"]:
            _field(step, key, where)
        if "fixed_ms" not in info and "timed_by" not in info:
            _field(step, "timeout_ms", where)
    _field(step, "volts", where)
    if kind == "turn_to_point" and step["volts"] != 12:
        raise ValueError(f"{where}: turnToPoint() always uses up to 12 V")
    if step.get("min_volts") is not None:
        _field(step, "min_volts", where)
        if step["min_volts"] > step["volts"]:
            raise ValueError(f"{where}: Min voltage ({step['min_volts']:g} V) must not be more than Max voltage "
                             f"({step['volts']:g} V)")
    if kind == "wall_reset" and index != count - 1:
        raise ValueError(f"{where}: a wall reset must be the last step, because it moves the odometry frame")
    step["actions"] = _validate_actions(step["actions"], where)
    if kind == "match_load" and field != "override":
        raise ValueError(f"{where}: Match Loads are part of the Override field. Choose the Override field")
    uses_mechanism = kind in MECHANISM_STEPS or step["actions"]
    if uses_mechanism and field != "override":
        raise ValueError(f"{where}: only the Override robot has a lift, a claw and a roller. Choose the Override field")
    if step["actions"] and kind in STILL_STEPS:
        raise ValueError(f"{where}: mechanism actions go on drive steps")
    if not isinstance(step["chain"], bool):
        raise ValueError(f"{where}: {CHAIN['label']} must be true or false")
    if step["chain"] and kind in CHAIN["not_allowed"]:
        raise ValueError(f"{where}: cannot keep moving into the next step. {CHAIN['not_allowed'][kind]}")
    if step["chain"] and index == count - 1:
        raise ValueError(f"{where}: cannot keep moving into the next step. {CHAIN['last_step']}")
    return step


def carries_speed(project, index):
    """True when path step @p index keeps moving into a path that sets off in
    the same direction, so the speed can carry across the join."""
    steps = project["steps"]
    if not steps[index]["chain"] or index + 1 == len(steps):
        return False
    step, following = steps[index], steps[index + 1]
    return (step["type"] == following["type"] == "path" and bool(step["points"]) and bool(following["points"])
            and step["sections"][-1]["reverse"] == following["sections"][0]["reverse"])


def step_time_limit(step, plan):
    """The step's own limit, or for a path left on automatic, the planned
    time plus room for the final turn and settle. A path that keeps moving
    does not settle, so it only gets a small margin."""
    if step["timeout_ms"] is not None:
        return step["timeout_ms"]
    allowance = LIMITS["chain_allowance_ms"] if step["chain"] else LIMITS["settle_allowance_ms"]
    planned_ms = plan["duration_s"] * 1000 + allowance
    return int(math.ceil(planned_ms / 100) * 100)


#: Back to the red Alliance Station wall, midway between the two stacks of Cups on it, 1 in off it so the Toggle
#: arms' tips are 0.5 in in front of the left Toggle and clear of it
OVERRIDE_START = dict(x=0, y=-61.7, heading=0)


def default_project():
    return copy.deepcopy(dict(DEFAULT_PROJECT, field=NEW_ROUTINE_FIELD, start=OVERRIDE_START, preload="yellow",
                              robot=dict(DEFAULT_PROJECT["robot"], preset=OVERRIDE_PRESET,
                                         tracking=OVERRIDE_TRACKING)))


def config():
    """Everything the page needs to draw its forms."""
    return dict(format=FORMAT, version=VERSION, game_fields=GAME_FIELDS, presets=PRESETS, tracking=TRACKING,
                heading_sources=HEADING_SOURCES, gains=GAINS, slew=SLEW, fields=FIELDS, step_types=STEP_TYPES, chain=CHAIN,
                actions=ACTIONS, preload=PRELOAD,
                path_defaults=PATH_DEFAULTS, conditions=CONDITIONS, limits=LIMITS,
                default_project=default_project())
