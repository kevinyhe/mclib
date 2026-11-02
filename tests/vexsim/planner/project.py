"""The path planner's project file: what it holds, its defaults, and the checks
that turn a bad file into a message naming the step and the field."""
import copy
import json
import math

FORMAT = "mclib-path-planner"
VERSION = 1

PRESETS = {
    "six_motor_450": dict(label="6 motors, 450 RPM, 3.25 in wheels", track_width_in=11.5,
                          body_length_in=15.0, body_width_in=12.5),
    "four_motor_200": dict(label="4 motors, 200 RPM, 3.25 in wheels", track_width_in=11.5,
                           body_length_in=15.0, body_width_in=12.5),
    "speed_base": dict(label="4 motors, 450 RPM, 4 in wheels", track_width_in=11.0,
                       body_length_in=15.0, body_width_in=12.5),
}
TRACKING = {
    "two": "Two tracking wheels (forward and sideways)",
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
    "timeout_ms": dict(label="Time limit", unit="ms", low=50, high=20000, step=50,
                       help="The step fails if it has not finished by then."),
    "volts": dict(label="Max voltage", unit="V", low=.1, high=12, step=.1,
                  help="Caps the drive voltage. 12 V is full power."),
}

STEP_TYPES = {
    "path": dict(label="Path", call="followTrajectory()",
                 description="Follow a curve through the points. mclib plans the speeds, then RAMSETE "
                             "steers the robot along the plan from odometry.",
                 fields=[], defaults=dict(points=[], sections=[], start_handle=None, timeout_ms=None, volts=12)),
    "drive": dict(label="Drive straight", call="driveTo()",
                  description="Drive a distance along the current heading.",
                  fields=["distance"], defaults=dict(distance=24)),
    "turn": dict(label="Turn to heading", call="turnToAngle()",
                 description="Turn in place to face a field heading.",
                 fields=["heading"], defaults=dict(heading=90)),
    "turn_to_point": dict(label="Turn to face a point", call="turnToPoint()",
                          description="Turn in place to face a field position. Backward points the back of the robot at it.",
                          fields=["x", "y", "direction"], defaults=dict(x=24, y=24, direction=1)),
    "point": dict(label="Drive to a point", call="moveToPoint()",
                  description="Turn toward a field position and drive to it.",
                  fields=["x", "y", "direction"], defaults=dict(x=24, y=24, direction=1)),
    "boomerang": dict(label="Drive to a pose", call="boomerang()",
                      description="Drive to a field position and arrive facing a heading, aiming at a moving point "
                                  "behind the target.",
                      fields=["x", "y", "heading", "direction", "lead"],
                      defaults=dict(x=24, y=24, heading=90, direction=1, lead=.5)),
    "arc": dict(label="Arc", call="curveCircle()",
                description="Drive around a circle of the given radius until facing the heading.",
                fields=["heading", "radius"], defaults=dict(heading=90, radius=24)),
    "reverse_arc": dict(label="Reverse arc", call="curveCircleReverse()",
                        description="Back around a circle of the given radius until facing the heading. "
                                    "The radius sign and the heading change must describe backing up.",
                        fields=["heading", "radius"], defaults=dict(heading=-90, radius=24)),
    "swing": dict(label="Swing turn", call="swing()",
                  description="Turn by driving one side while the other side holds still.",
                  fields=["heading", "direction"], defaults=dict(heading=90, direction=1)),
    "wall_reset": dict(label="Wall reset", call="wallReset()",
                       description="Push into a wall until the motors stall, then set odometry to this pose. "
                                   "The simulator has no walls: the drive is held still as if it were touching one. "
                                   "Must be the last step, because it moves the odometry frame.",
                       fields=["x", "y", "heading", "current_ma"],
                       defaults=dict(x=0, y=0, heading=0, current_ma=2500)),
}

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
              max_handle_in=300, settle_allowance_ms=4000)

DEFAULT_PROJECT = dict(
    format=FORMAT, version=VERSION, name="New routine",
    robot=dict(preset="six_motor_450", tracking="two", heading_source="imu"),
    conditions={key: item["default"] for key, item in CONDITIONS.items()},
    gains={key: item["default"] for key, item in GAINS.items()},
    path_defaults={key: item["default"] for key, item in PATH_DEFAULTS.items()},
    start=dict(x=0, y=-48, heading=0),
    stop_on_failure=True,
    steps=[dict(type="path", **copy.deepcopy(STEP_TYPES["path"]["defaults"]))],
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
    _number(value, info["low"], info["high"], f"{where}: {info['label']}", info["unit"],
            integer=key == "timeout_ms")
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
    _object(raw, set(DEFAULT_PROJECT), "The project")
    if raw.get("format", FORMAT) != FORMAT:
        raise ValueError(f"This is not a path planner project (format is \"{raw.get('format')}\")")
    if raw.get("version", VERSION) != VERSION:
        raise ValueError(f"This project is version {raw.get('version')}; this planner reads version {VERSION}")
    project = copy.deepcopy(DEFAULT_PROJECT)
    for key in ("name", "stop_on_failure"):
        if key in raw:
            project[key] = raw[key]
    if not isinstance(project["name"], str) or not 1 <= len(project["name"]) <= 80 or \
            any(ord(c) < 32 for c in project["name"]):
        raise ValueError("The routine name must be 1 to 80 characters on one line")
    if not isinstance(project["stop_on_failure"], bool):
        raise ValueError("Stop at the first failed step must be true or false")
    project["robot"].update(_object(raw.get("robot", {}), project["robot"], "Robot"))
    for key, choices in (("preset", PRESETS), ("tracking", TRACKING), ("heading_source", HEADING_SOURCES)):
        if project["robot"][key] not in choices:
            raise ValueError(f"Robot {key.replace('_', ' ')} must be one of: {', '.join(choices)}")
    for group, schema, title in (("conditions", CONDITIONS, "Conditions"), ("gains", GAINS, "Controller gains"),
                                 ("path_defaults", PATH_DEFAULTS, "Path settings")):
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
            project["steps"].append(_validate_step(index, value, len(steps)))
        except ValueError as error:
            raise ProjectError(str(error), index) from error
    return project


def _validate_step(index, value, count):
    where = step_name(index, value)
    if not isinstance(value, dict) or value.get("type") not in STEP_TYPES:
        raise ValueError(f"Step {index + 1}: unknown step type {value.get('type') if isinstance(value, dict) else value!r}")
    kind = value["type"]
    info = STEP_TYPES[kind]
    _object(value, set(info["defaults"]) | {"type", "note", "timeout_ms", "volts"}, where)
    step = dict(type=kind, note="", timeout_ms=4000, volts=6 if kind == "wall_reset" else 12)
    step.update(copy.deepcopy(info["defaults"]))
    step.update(value)
    if not isinstance(step["note"], str) or len(step["note"]) > 80:
        raise ValueError(f"{where}: the note must be text of up to 80 characters")
    if kind == "path":
        _validate_path(value, step, where)
    else:
        for key in info["fields"] + ["timeout_ms"]:
            _field(step, key, where)
    _field(step, "volts", where)
    if kind == "turn_to_point" and step["volts"] != 12:
        raise ValueError(f"{where}: turnToPoint() always uses up to 12 V")
    if kind == "wall_reset" and index != count - 1:
        raise ValueError(f"{where}: a wall reset must be the last step, because it moves the odometry frame")
    return step


def step_time_limit(step, plan):
    """The step's own limit, or for a path left on automatic, the planned
    time plus room for the final turn and settle."""
    if step["timeout_ms"] is not None:
        return step["timeout_ms"]
    planned_ms = plan["duration_s"] * 1000 + LIMITS["settle_allowance_ms"]
    return int(math.ceil(planned_ms / 100) * 100)


def default_project():
    return copy.deepcopy(DEFAULT_PROJECT)


def config():
    """Everything the page needs to draw its forms."""
    return dict(format=FORMAT, version=VERSION, presets=PRESETS, tracking=TRACKING,
                heading_sources=HEADING_SOURCES, gains=GAINS, fields=FIELDS, step_types=STEP_TYPES,
                path_defaults=PATH_DEFAULTS, conditions=CONDITIONS, limits=LIMITS,
                default_project=default_project())
