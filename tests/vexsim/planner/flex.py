"""The Override robot's lift and claw in the planner.

flex_helpers.hpp holds mclib PresetPositionMechanisms for the arm and the
claw. Here their sensor reads and motor writes are connected to the
simulated motors of vexsim's OverrideWorld, and Pins and Cups are turned
into planner coordinates for the page.
"""
import ctypes as C
import math

from fields import piece_pose, to_planner

ARM_PRESETS = {
    "down": "Down (0°)",
    "into_alliance_goal": "Into an Alliance Goal (290°)",
    "above_alliance_goal": "Above an Alliance Goal (520°)",
    "above_neutral_goal": "Above a Neutral Goal (820°)",
    "above_center_goal": "Above the Center Goal (1200°)",
    "top": "Top (2000°)",
}
CLAW_PRESETS = {"open": "Open (60°)", "closed": "Closed (467°)"}
MECHANISMS = {"arm": dict(label="Lift", presets=ARM_PRESETS, enum="flex::ArmPreset"),
              "claw": dict(label="Claw", presets=CLAW_PRESETS, enum="flex::ClawPreset")}
ACTION_TIMES = {"start": "When the step starts", "distance": "After driving", "time": "After"}
ACTION_UNITS = {"start": "", "distance": "in", "time": "ms"}
PRELOADS = {"none": "No preload", "yellow": "Red Pin, yellow half up", "red": "Red Pin, red half up"}
MOVE_RESULTS = ("Used the whole time limit", "Reached the preset", "Stopped closing on a game object")

READ_DEGREES = C.CFUNCTYPE(C.c_double, C.c_int)
WRITE_VOLTS = C.CFUNCTYPE(None, C.c_int, C.c_double)


def cpp_preset(mechanism, preset):
    words = preset.split("_")
    return MECHANISMS[mechanism]["enum"] + "::" + "".join(word.capitalize() for word in words)


def preset_number(mechanism, preset):
    return list(MECHANISMS[mechanism]["presets"]).index(preset)


def declare_flex(lib):
    doubles = C.POINTER(C.c_double)
    lib.planner_flex_init.argtypes = [READ_DEGREES, WRITE_VOLTS]
    lib.planner_flex_init.restype = None
    lib.planner_flex_begin_actions.argtypes = [doubles, C.c_int]
    lib.planner_flex_begin_actions.restype = None
    lib.planner_flex_finish_actions.argtypes = []
    lib.planner_flex_finish_actions.restype = C.c_int
    lib.planner_flex_move.argtypes = [C.c_int, C.c_int, C.c_int]
    lib.planner_flex_move.restype = C.c_int


class FlexRig:
    """flex::Rig in the C++, reading and driving the world's arm and claw
    motors. Errors in the callbacks go to the bridge, which cancels the
    running motion and raises once C++ returns."""

    def __init__(self, lib, bridge, world):
        self.lib = lib
        self.bridge = bridge
        self.motors = (world.mechanism.arm, world.mechanism.claw)
        self.read_callback = READ_DEGREES(self.read)
        self.write_callback = WRITE_VOLTS(self.write)
        lib.planner_flex_init(self.read_callback, self.write_callback)

    def read(self, motor):
        try:
            return math.degrees(self.motors[motor].position)
        except Exception as error:
            self.bridge.exceptions.append(str(error))
            return math.nan

    def write(self, motor, volts):
        try:
            if not math.isfinite(volts):
                raise ValueError(f"invalid {('arm', 'claw')[motor]} voltage {volts}")
            self.motors[motor].set_voltage(max(-12.0, min(12.0, volts)))
        except Exception as error:
            self.bridge.exceptions.append(str(error))

    def move(self, mechanism, preset, timeout_ms):
        result = self.lib.planner_flex_move(int(mechanism == "arm"), preset_number(mechanism, preset), timeout_ms)
        self.raise_errors()
        return result

    def begin_actions(self, actions):
        values = [value for action in actions for value in
                  (list(ACTION_TIMES).index(action["when"]), action["value"], int(action["mechanism"] == "arm"),
                   preset_number(action["mechanism"], action["preset"]))]
        self.lib.planner_flex_begin_actions((C.c_double * max(1, len(values)))(*values), len(actions))

    def finish_actions(self):
        late = self.lib.planner_flex_finish_actions()
        self.raise_errors()
        return late

    def raise_errors(self):
        if self.bridge.exceptions:
            raise RuntimeError("; ".join(self.bridge.exceptions))


def frame_telemetry(world):
    """The world's telemetry in planner coordinates, with the score so far."""
    frame = world.telemetry()
    frame["pieces"] = {str(key): piece_pose(values) for key, values in frame["pieces"].items()}
    frame["holding"] = None if frame["holding"] is None else describe_piece(world, frame["holding"])
    score = world.score(autonomous=True)
    frame.update(score_red=score["red"].total, score_blue=score["blue"].total)
    return frame


def describe_piece(world, piece_id):
    piece = next(p for p in world.pieces.pieces() if p.id == piece_id)
    if piece.kind == "cup":
        return "a Cup"
    first, second = piece.colors
    return f"a {first} Pin" if first == second else f"a {first}/{second} Pin"


def claw_events(world, start, end):
    """Info lines for what the claw picked up and let go of during a step."""
    from vexsim.units import INCH
    events = [(grip["t"], "Gripped", grip, None) for grip in world.grips] + \
             [(drop["t"], "Dropped", drop, drop["reason"]) for drop in world.drops]
    lines = []
    for t, verb, event, reason in sorted(events, key=lambda item: item[0]):
        if not start <= t <= end or t == 0:
            continue
        x, y = to_planner(event["at"][0], event["at"][1])
        height = event["at"][2] / INCH
        line = (f"{verb} {describe_piece(world, event['piece'])} at ({x:.0f}, {y:.0f}), {height:.1f} in up, "
                f"{t - start:.2f} s into the step")
        lines.append(line + (f": {reason}." if reason else "."))
    return lines


def score_summary(world):
    score = world.score(autonomous=True)
    return {alliance: dict(alliance_pins=item.alliance_pins, yellow_pins=item.yellow_pins, total=item.total)
            for alliance, item in score.items()}
