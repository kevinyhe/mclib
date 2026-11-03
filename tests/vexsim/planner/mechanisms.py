"""The Override robot's cascade lift, claw and Toggle arm in the planner.

cascade_helpers.hpp holds an mclib PresetPositionMechanism for each. Here
their sensor reads and motor writes are connected to the simulated motors of
vexsim's cascade robot, and Pins and Cups are turned into planner coordinates
for the page.
"""
import ctypes as C
import math

from fields import piece_pose, to_planner

LIFT_PRESETS = {
    "down": "Down: claw 4.2 in up",
    "pin_in_cup": "At a Pin standing in a Cup: claw 6.8 in up",
    "over_alliance_goal": "Over an Alliance Goal: claw 6.65 in up",
    "over_neutral_goal": "Over a Neutral Goal: claw 9.17 in up",
    "over_center_goal": "Over the Center Goal: claw 12.17 in up",
    "top": "Top: claw 40.2 in up",
}
#: The claw heights the presets name, inches: cascade_helpers.hpp holds the same numbers
LIFT_HEIGHTS_IN = dict(down=4.2, pin_in_cup=6.8, over_alliance_goal=6.65, over_neutral_goal=9.17,
                       over_center_goal=12.17, top=40.2)
CLAW_PRESETS = {"open": "Open: jaws 4 in apart (0°)", "closed": "Closed: squeezing (180°)"}
TOGGLE_ARM_PRESETS = {
    "stowed": "Stowed: folded forward over the robot (0°)",
    "over_the_axle": "Over the axle: pad behind the robot, 0.95 in over a Toggle's axle (464.5°)",
    "under_the_axle": "Under the axle: pad behind the robot, 0.75 in under a Toggle's axle (540°)",
}
MECHANISMS = {
    "lift": dict(label="Lift", presets=LIFT_PRESETS, enum="cascade::LiftPreset", method="moveLift"),
    "claw": dict(label="Claw", presets=CLAW_PRESETS, enum="cascade::ClawPreset", method="moveClaw"),
    "toggle_arm": dict(label="Toggle arm", presets=TOGGLE_ARM_PRESETS, enum="cascade::ToggleArmPreset",
                       method="moveToggleArm"),
}
MECHANISM_ENUMS = {"lift": "cascade::Mechanism::Lift", "claw": "cascade::Mechanism::Claw",
                   "toggle_arm": "cascade::Mechanism::ToggleArm"}
#: Steps and actions from files made for the Flex robot, which had an inclined lift called the arm
RETIRED_MECHANISMS = {"arm": "lift"}
ACTION_TIMES = {"start": "When the step starts", "distance": "After driving", "time": "After",
                "end": "When the motion ends"}
ACTION_UNITS = {"start": "", "distance": "in", "time": "ms", "end": ""}
PRELOADS = {"none": "No preload", "yellow": "Red Pin, yellow half up", "red": "Red Pin, red half up"}
MOVE_RESULTS = ("Used the whole time limit", "Reached the preset", "Stopped closing on a game object")
#: <SC7>: the Autonomous Bonus goes to whoever leads when the period ends
AUTONOMOUS_S = 15.0
#: <SG1>, <SG2>, <SG3>, <SG6>, <R10>, <R11>
START_SIZE_IN = 18.0
MAX_SIZE_IN = 24.0
MAX_HEIGHT_IN = 50.0
MAX_MOTOR_WATTS = 88.0
MAX_DRIVE_WATTS = 55.0

READ_DEGREES = C.CFUNCTYPE(C.c_double, C.c_int)
WRITE_VOLTS = C.CFUNCTYPE(None, C.c_int, C.c_double)


def cpp_preset(mechanism, preset):
    words = preset.split("_")
    return MECHANISMS[mechanism]["enum"] + "::" + "".join(word.capitalize() for word in words)


def preset_number(mechanism, preset):
    return list(MECHANISMS[mechanism]["presets"]).index(preset)


def mechanism_number(mechanism):
    return list(MECHANISMS).index(mechanism)


def declare_cascade(lib):
    doubles = C.POINTER(C.c_double)
    lib.planner_cascade_init.argtypes = [READ_DEGREES, WRITE_VOLTS]
    lib.planner_cascade_init.restype = None
    lib.planner_cascade_begin_actions.argtypes = [doubles, C.c_int]
    lib.planner_cascade_begin_actions.restype = None
    lib.planner_cascade_finish_actions.argtypes = []
    lib.planner_cascade_finish_actions.restype = C.c_int
    lib.planner_cascade_move.argtypes = [C.c_int, C.c_int, C.c_int]
    lib.planner_cascade_move.restype = C.c_int


class CascadeRig:
    """cascade::Rig in the C++, reading and driving the world's lift, claw
    and Toggle arm motors. The two lift motors share a shaft: the rig reads
    one and drives both. Errors in the callbacks go to the bridge, which
    cancels the running motion and raises once C++ returns."""

    def __init__(self, lib, bridge, world):
        self.lib = lib
        self.bridge = bridge
        mechanism = world.mechanism
        self.motors = (mechanism.lift.motors, [mechanism.claw], [mechanism.toggle_arm.motor])
        self.read_callback = READ_DEGREES(self.read)
        self.write_callback = WRITE_VOLTS(self.write)
        lib.planner_cascade_init(self.read_callback, self.write_callback)

    def read(self, mechanism):
        try:
            return math.degrees(self.motors[mechanism][0].position)
        except Exception as error:
            self.bridge.exceptions.append(str(error))
            return math.nan

    def write(self, mechanism, volts):
        try:
            if not math.isfinite(volts):
                raise ValueError(f"invalid {list(MECHANISMS)[mechanism]} voltage {volts}")
            for motor in self.motors[mechanism]:
                motor.set_voltage(max(-12.0, min(12.0, volts)))
        except Exception as error:
            self.bridge.exceptions.append(str(error))

    def move(self, mechanism, preset, timeout_ms):
        result = self.lib.planner_cascade_move(mechanism_number(mechanism), preset_number(mechanism, preset),
                                               timeout_ms)
        self.raise_errors()
        return result

    def begin_actions(self, actions):
        values = [value for action in actions for value in
                  (list(ACTION_TIMES).index(action["when"]), action["value"], mechanism_number(action["mechanism"]),
                   preset_number(action["mechanism"], action["preset"]))]
        self.lib.planner_cascade_begin_actions((C.c_double * max(1, len(values)))(*values), len(actions))

    def finish_actions(self):
        late = self.lib.planner_cascade_finish_actions()
        self.raise_errors()
        return late

    def raise_errors(self):
        if self.bridge.exceptions:
            raise RuntimeError("; ".join(self.bridge.exceptions))


def mechanism_reading(world, mechanism):
    """(what it is at, in words; the number) for a mechanism step's message."""
    from vexsim.units import INCH
    robot = world.robot
    if mechanism == "lift":
        height = robot.grip_height() / INCH
        return f"the claw is {height:.2f} in up", height
    if mechanism == "claw":
        gap = world.mechanism.gap / INCH
        return f"the jaws are {gap:.2f} in apart", gap
    degrees = math.degrees(world.mechanism.toggle_arm.motor.position)
    return f"the Toggle arm motor is at {degrees:.0f}°", degrees


def frame_telemetry(world):
    """The world's telemetry in planner coordinates, with the score so far."""
    frame = world.telemetry()
    frame["pieces"] = {str(key): piece_pose(values) for key, values in frame["pieces"].items()}
    frame["holding"] = None if frame["holding"] is None else describe_held(world)
    score = world.score(autonomous=True)
    frame.update(score_red=score["red"].total, score_blue=score["blue"].total)
    return frame


def describe_piece(world, piece_id):
    piece = next(p for p in world.pieces.pieces() if p.id == piece_id)
    if piece.kind == "cup":
        return "a Cup"
    first, second = piece.colors
    return f"a {first} Pin" if first == second else f"a {first}/{second} Pin"


def describe_held(world):
    """The held piece, and what is stacked with it."""
    held = world.grip.piece
    others = [p for p in held.body.pieces if p is not held]
    text = describe_piece(world, held.id)
    return text + "".join(f" with {describe_piece(world, p.id)}" for p in others)


def possession(world):
    """(Pins, Cups) the robot holds: the gripped piece and everything stacked with it (<SG6>)."""
    if world.grip is None:
        return 0, 0
    kinds = [piece.kind for piece in world.grip.piece.body.pieces]
    return kinds.count("pin"), kinds.count("cup")


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


def autonomous_summary(world, sizes):
    """The score as the autonomous period ends: totals, who gets the
    Autonomous Bonus, how close red is to the Autonomous Win Point, and the
    robot's size and motors against the rules."""
    from vexsim import override_field as of
    from vexsim.units import INCH
    score = world.score(autonomous=True)
    toggles = world.toggle_colors
    pins, goals = of.win_point_progress("red", world.stacks(), toggles)
    pins_needed, goals_needed = of.win_point_needs()
    touching = world.arena.touching_perimeter
    state = world.sim.state
    return dict(
        t=world.sim.t, red=score["red"].total, blue=score["blue"].total,
        bonus=of.autonomous_bonus(score), toggles=toggles,
        win_point=dict(pins=pins, pins_needed=pins_needed, goals_with_two=goals, goals_needed=goals_needed,
                       touching_perimeter=touching,
                       earned=pins >= pins_needed and goals >= goals_needed and not touching),
        robot_in_midfield=of.in_midfield(state.x, state.y),
        robot_at=to_planner(state.x, state.y), robot_inches_from_midfield=round(
            max(0.0, abs(state.x) + abs(state.y) - of.MIDFIELD_HALF_DIAGONAL) / INCH, 1),
        rules=sizes.report())


class RuleWatch:
    """Tracks the robot against the size and possession rules over a run:
    its size at the start (<SG1>), the longest, widest and tallest it gets
    (<SG2>, <SG3>), and the most Pins and Cups it holds at once (<SG6>)."""

    def __init__(self, world, drive_watts):
        from vexsim.units import INCH
        self.world = world
        self.inch = INCH
        self.start = self._extent()
        self.largest = list(self.start)
        self.held = [0, 0]
        self.drive_watts = drive_watts
        self.mechanism_watts = sum(world.robot.motor_watts().values())

    def _extent(self):
        return [value / self.inch for value in self.world.robot.extent(self.world.sim.chassis)]

    def watch(self):
        """The rule this frame breaks for the first time, in words, or None."""
        extent = self._extent()
        broken = None
        if max(extent[:2]) > MAX_SIZE_IN >= max(self.largest[:2]):
            broken = (f"The robot was {max(extent[:2]):.1f} in long or wide. It may never be more than "
                      f"{MAX_SIZE_IN:g} in (SG2).")
        if extent[2] > MAX_HEIGHT_IN >= self.largest[2]:
            broken = f"The robot was {extent[2]:.1f} in tall. It may never be taller than {MAX_HEIGHT_IN:g} in (SG3)."
        self.largest = [max(a, b) for a, b in zip(self.largest, extent)]
        pins, cups = possession(self.world)
        if (pins > 1 >= self.held[0]) or (cups > 1 >= self.held[1]):
            broken = (f"The robot held {pins} Pin{'s' * (pins != 1)} and {cups} Cup{'s' * (cups != 1)} at once. "
                      "It may hold one Pin and one Cup at most (SG6).")
        self.held = [max(self.held[0], pins), max(self.held[1], cups)]
        return broken

    def report(self):
        length, width, height = self.start
        return dict(
            start_in=dict(length=round(length, 2), width=round(width, 2), height=round(height, 2),
                          limit=START_SIZE_IN, passed=max(self.start) <= START_SIZE_IN),
            largest_in=dict(length=round(self.largest[0], 2), width=round(self.largest[1], 2),
                            height=round(self.largest[2], 2), size_limit=MAX_SIZE_IN, height_limit=MAX_HEIGHT_IN,
                            passed=max(self.largest[:2]) <= MAX_SIZE_IN and self.largest[2] <= MAX_HEIGHT_IN),
            most_held=dict(pins=self.held[0], cups=self.held[1], passed=self.held[0] <= 1 and self.held[1] <= 1),
            motors_w=dict(drive=self.drive_watts, total=self.drive_watts + self.mechanism_watts,
                          drive_limit=MAX_DRIVE_WATTS, total_limit=MAX_MOTOR_WATTS,
                          passed=self.drive_watts <= MAX_DRIVE_WATTS and
                          self.drive_watts + self.mechanism_watts <= MAX_MOTOR_WATTS))


def toggle_faces(world):
    return {bar.mount.quadrant: bar.inner_face() for bar in world.toggles.bars}


def toggle_events(world, before):
    """Info lines for Toggles whose face into the field changed in a step."""
    lines = []
    for bar in world.toggles.bars:
        quadrant, color = bar.mount.quadrant, bar.inner_face()
        if color == before[quadrant]:
            continue
        line = f"Turned the {quadrant} Toggle: {color} now faces into the field ({before[quadrant]} before)."
        if bar.touched:
            line += " The robot is still touching it, so it counts as yellow until the robot lets go."
        lines.append(line)
    return lines
