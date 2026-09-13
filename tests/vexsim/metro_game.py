"""One-clock native Push Back coupling for the external Metro C++ harness.

The caller owns Simulator creation, its physical configuration/start pose, all
drive outputs and C++ sensor/odometry timing. This helper never runs a Python
controller, starts a thread, or substitutes Python odometry for Metro's pose.

Mechanism fidelity is deliberately narrower than field/contact fidelity. Native
CAD intake stages are a constrained-path approximation, with command-scaled
surface speeds, not Metro motor electrical dynamics or measured robot geometry.
"""

from copy import deepcopy
import math
from types import SimpleNamespace

from vexsim.game import BLOCK_HALF, Block, Game, Intake
from vexsim.field import from_corner
from vexsim.units import G, INCH, LB


_STAGE_NAMES = (
    "front flex wheels", "second flex wheels", "bottom sprocket",
    "top sprocket", "upper flex wheels", "exit flex wheels",
)
_WARNINGS = (
    "Native Push Back field/contact physics; custom Metro mechanism scoring is approximate.",
    "Intake uses the native CAD constrained route and command-scaled roller speeds, not roller contacts or Metro motor electrical loads.",
    "Flap-backed middle scoring selects the native center route; native route changes re-seat held blocks on that route.",
    "Motor signs are mapped into route travel: middle reverses upper rollers; low scoring reverses the front route. PRIME backs blocks down the long route with the gate closed.",
    "Scraper and wing outputs are recorded only: their geometry, deployment contacts and pneumatic dynamics are unsupported.",
    "One native robot is simulated; station match loads and the other robots/preloads are not automatically spawned.",
)


def _finite(value, name):
    if isinstance(value, bool) or not isinstance(value, (float, int)) or not math.isfinite(value):
        raise ValueError(f"{name} must be finite")
    return float(value)


def _safe_json(value):
    """Detach supplied telemetry; unavailable numeric readings stay JSON null."""
    if isinstance(value, float) and not math.isfinite(value):
        return None
    if isinstance(value, dict):
        return {str(key): _safe_json(item) for key, item in value.items()}
    if isinstance(value, (list, tuple)):
        return [_safe_json(item) for item in value]
    if value is None or isinstance(value, (bool, int, float, str)):
        return value
    raise ValueError("telemetry must contain only JSON-compatible values")


def _mean(values):
    values = list(values)
    return sum(values) / len(values) if values else 0.0


def competition_starting_blocks():
    """36 floor blocks: official Push Back 4.0 Appendix A, pp. A-5–A-7.

    Coordinates are inches from the audience-left corner on drawing 276-9142.
    Under-goal blocks are obscured in the top drawing; their four-per-goal
    arrangement is also present in the supplied official field CAD.
    """
    rows = []
    for colour, outer, inner in (("red", 46.65, 49.88), ("blue", 93.80, 90.57)):
        for edge, inward in ((46.63, 49.88), (93.78, 90.53)):
            rows.extend((x, y, colour) for x, y in
                        ((outer, edge), (inner, edge), (outer, inward)))
    for y in (23.44, 116.97):
        rows.extend((x, y, colour) for colour, xs in
                    (("red", (65.38, 68.61)), ("blue", (71.84, 75.07))) for x in xs)
    for colour, xs in (("blue", (21.46, 24.69)), ("red", (115.76, 118.99))):
        rows.extend((x, y, colour) for x in xs for y in (1.61, 138.80))
    for colour, x in (("blue", 14.03), ("red", 126.42)):
        rows.extend((x, y, colour) for y in (65.36, 68.59, 71.82, 75.05))
    return [(*from_corner(x, y), colour) for x, y, colour in rows]


class _CommandedGame(Game):
    """Retain native Game maths, but do not power rollers merely to hold blocks.

    Native Game.step turns its rollers on whenever something is held. Metro's
    measured outputs instead decide whether native staged traction is powered.
    Zero-command stages additionally have zero grip: unpowered motor braking is
    not characterized here, so it must not be invented as active holding torque.
    Gravity, gate constraints, block contacts and passive route motion remain.
    """

    commanded_power = False
    matchload_rate = 1.0
    goal_stopping_rate = 1.0

    def _friction(self, blocks, dt):
        # Scale only supported goal-contact losses (sliding and rattle).
        # Positions, collision impulses and the simulation clock keep real dt.
        goals = [b for b in blocks if b.support == "channel"]
        others = [b for b in blocks if b.support != "channel"]
        super()._friction(others, dt)
        super()._friction(goals, dt * self.goal_stopping_rate)

    def _take(self, block):
        from_loader = block.state == "loader"
        super()._take(block)
        if from_loader:
            self.intake_timer /= self.matchload_rate

    def _drive_held(self, dt):
        self.rollers_on = self.commanded_power
        super()._drive_held(dt)

    def step(self, dt):
        super().step(dt)
        self.rollers_on = self.commanded_power


class MetroGame:
    """Attach native Game to an existing, externally driven Simulator.

    ``alliance``, deterministic game ``seed`` and ``preloads`` are explicit.
    Preloads may be an alliance-coloured count or a sequence of red/blue names.
    No physical start-pose default is imposed and construction advances no time.

    ``snapshot`` requires the real Metro pose as (x inches, y inches, CW degrees),
    in the same +Y-forward/+X-right frame used by the mclib bridge. The returned
    native odom fields are only a coordinate conversion of that supplied pose.
    """

    def __init__(self, sim, *, alliance, seed, preloads, assumptions=None):
        if alliance not in ("red", "blue"):
            raise ValueError("alliance must be red or blue")
        if isinstance(seed, bool) or not isinstance(seed, int):
            raise ValueError("seed must be an integer")
        intake = deepcopy(Intake())  # native dataclass roller defaults are shared
        if tuple(stage.name for stage in intake.rollers) != _STAGE_NAMES:
            raise RuntimeError("unsupported native intake stage layout")
        if isinstance(preloads, int) and not isinstance(preloads, bool):
            if not 0 <= preloads <= intake.capacity:
                raise ValueError("preload count exceeds native intake capacity")
            colours = [alliance] * preloads
        elif isinstance(preloads, (tuple, list)):
            colours = list(preloads)
        else:
            raise ValueError("preloads must be a count or colour sequence")
        if len(colours) > intake.capacity or any(c not in ("red", "blue") for c in colours):
            raise ValueError("preloads must fit the native intake and be red or blue")
        self.sim = sim
        self.alliance = alliance
        self.seed = seed
        self.preload_colours = colours
        self.assumptions = _safe_json(assumptions if assumptions is not None else {
            "physical_source": "Caller-supplied chassis; native CAD intake/body is an unmeasured fallback, not verified Metro geometry.",
            "motor_model": "Native V5 model; custom mechanism motors do not contribute current or battery load.",
        })
        self.game = _CommandedGame(SimpleNamespace(sim=sim, chassis=sim.chassis),
                                  alliance=alliance, seed=seed, intake=intake)
        floor_blocks = [b for b in self.game.blocks if b.state == "field"]
        layout = competition_starting_blocks()
        if len(floor_blocks) != len(layout):
            raise RuntimeError("Unsupported native floor block count")
        for block, (x, y, colour) in zip(floor_blocks, layout):
            block.x, block.y, block.colour, block.yaw = x, y, colour, 0.0
        for index, loader in enumerate(self.game.field.loaders):
            stack = sorted((b for b in self.game.blocks if b.state == "loader" and b.loader == index),
                           key=lambda b: b.z)
            for level, block in enumerate(stack):
                block.colour = loader.alliance if level < 3 else ("blue" if loader.alliance == "red" else "red")
        self._nominal_stages = tuple((r.rpm, r.grip) for r in intake.rollers)
        self._stage_angles = [0.0] * len(intake.rollers)
        # Initialization only: preserve every field/loader object, replacing
        # the native one-preload setup with the caller's explicit selection.
        self.game.blocks = [b for b in self.game.blocks if b.state != "held"]
        stop = intake.stop_at()
        self._default_optical = intake.at_distance(stop)
        self.game.held = [Block(x=0.0, y=0.0, colour=c, state="held",
                                ride=stop + i * intake.pitch)
                          for i, c in enumerate(colours)]
        self.game.blocks.extend(self.game.held)
        # Zero-speed placement uses native route geometry, without a hidden 4 ms
        # warm-up tick or movement/creation of field blocks.
        self.game._place_held()
        self.wall_contact = False
        self.trail = []
        self.events = []
        self.outputs = None
        self.reverse_transport_unsupported = False
        self.set_outputs(0, 0, False, False, state="DISABLED")

    def start(self):
        """Start the native match clock, without stepping or enabling motors."""
        self.game.start()

    def set_outputs(self, bottom, top, hood, flap, *, scraper=False, wing=False,
                    state=None):
        """Record actual Metro outputs and apply the declared native approximation.

        ``bottom``/``top`` are PROS move commands in [-127,127], not measured RPM.
        Lower four native stages follow bottom; upper two follow top. Their
        signed target surface speeds are proportional to those commands. Mixed
        Electrical signs are recorded unchanged. Roller surface directions
        are expressed along the selected route, which reverses for low scoring
        and passes under the upper rollers for middle scoring.
        """
        bottom = _finite(bottom, "bottom")
        top = _finite(top, "top")
        if abs(bottom) > 127 or abs(top) > 127:
            raise ValueError("mechanism motor commands must be in [-127, 127]")
        for name, value in (("hood", hood), ("flap", flap),
                            ("scraper", scraper), ("wing", wing)):
            if type(value) is not bool:
                raise ValueError(f"{name} must be a boolean")
        if state is not None and (isinstance(state, bool) or not isinstance(state, (str, int))):
            raise ValueError("state must be a string, integer enum or null")
        outputs = dict(bottom=bottom, top=top, hood=hood, flap=flap,
                       scraper=scraper, wing=wing, state=state)
        if outputs == self.outputs:
            return
        self.outputs = outputs
        self.events.append({"t": self.sim.t, "outputs": dict(outputs)})
        prime = state in ("MIDDLE_GOAL_PRIME", 2)
        reverse = bottom < 0 and top <= 0 and not hood and not flap and not prime
        middle = bottom > 0 and top < 0 and flap
        long_score = bottom > 0 and top > 0 and hood and not flap
        route = "lower" if reverse else "center" if middle else "long"
        self.reverse_transport_unsupported = False
        # Native traction expects positive speed toward the route's exit.
        # Electrical reversal moves forwards on the reversed/under-roller path.
        commands = (-bottom, -top) if reverse else (bottom, -top) if middle else (bottom, top)
        for i, (stage, (rpm, grip)) in enumerate(zip(self.game.intake.rollers,
                                                     self._nominal_stages)):
            command = commands[0 if i < 4 else 1]
            stage.rpm = rpm * command / 127.0
            stage.grip = grip if command != 0 else 0.0
        self.game.commanded_power = any(command != 0 for command in commands)
        self.game.rollers_on = self.game.commanded_power
        self.game.intake_running = bottom > 0
        self.game.scoring = reverse or middle or long_score
        # A route switch is the native constrained-intake approximation only;
        # it never relocates a field block or inserts a block into a Goal.
        if self.game.scoring or prime or (bottom > 0 and not hood and not flap):
            self.game.set_route(route)
        # Native prediction cache excludes RPM, so invalidate on output changes.
        self.game._exit_cache = ((), 0.0)

    def optical_sample(self, *, forward_in=None, lateral_in=0.0, height_in=None,
                       detection_radius_in=2.5, max_distance_mm=2000.0):
        """Configurable geometric proxy, NOT calibrated Metro optical hardware.

        The default fixed sensor location is the native long-route gate's held
        block center at construction. It does not move when routes switch. Any
        actual native block center in the configured spherical detection region
        reads occupied (255); colour supplies hue. There is no scripted clearing
        of a held preload during PRIME. Distance is the geometric distance from
        the sensor point to the nearest native axis-aligned block box, not a
        raycast or a measured optical/distance transfer function.
        """
        forward = (self._default_optical[0] if forward_in is None
                   else _finite(forward_in, "forward_in") * INCH)
        height = (self._default_optical[1] if height_in is None
                  else _finite(height_in, "height_in") * INCH)
        lateral = _finite(lateral_in, "lateral_in") * INCH
        radius = _finite(detection_radius_in, "detection_radius_in") * INCH
        maximum = _finite(max_distance_mm, "max_distance_mm")
        if not 0 < radius <= 24 * INCH or not 0 < maximum <= 10000:
            raise ValueError("invalid geometric sensor detection bounds")
        if abs(forward) > 200 * INCH or abs(lateral) > 200 * INCH or not 0 <= height <= 200 * INCH:
            raise ValueError("invalid body-relative geometric sensor position")
        st = self.sim.state
        c, s = math.cos(st.theta), math.sin(st.theta)
        x, y = st.x + c * forward - s * lateral, st.y + s * forward + c * lateral
        nearest = min(self.game.blocks, key=lambda b: math.dist((x, y, height),
                                                               (b.x, b.y, b.z)), default=None)
        distance = (math.dist((x, y, height), (nearest.x, nearest.y, nearest.z))
                    if nearest is not None else math.inf)
        occupied = distance <= radius
        surface_distance = min((math.sqrt(sum(max(0.0, abs(a - b) - BLOCK_HALF) ** 2
                                             for a, b in zip((x, y, height), (block.x, block.y, block.z))))
                                for block in self.game.blocks), default=maximum / 1000)
        return {
            "proximity": 255 if occupied else 0,
            "hue": (0.0 if nearest.colour == "red" else 210.0) if occupied else 0.0,
            "distance_mm": min(maximum, surface_distance * 1000),
            "detected_colour": nearest.colour if occupied else None,
            "detected_state": nearest.state if occupied else None,
            "model": "uncalibrated_native_block_geometry_proxy",
            "warning": "Optical placement/proximity/hue and distance response are assumptions, not Metro hardware validation; occupied blocks are never cleared by a scripted timer.",
            "body_position_in": {"forward": forward / INCH, "lateral": lateral / INCH,
                                 "height": height / INCH},
            "detection_radius_in": radius / INCH,
        }

    def step(self, dt=0.001):
        """Advance precisely one existing public physics tick, in native order."""
        dt = _finite(dt, "dt")
        if dt != 0.001:
            raise ValueError("MetroGame.step requires exactly one 0.001 s tick")
        self.sim.step(dt)
        self.wall_contact = self.game._perimeter()
        self.game.step(dt)
        for i, stage in enumerate(self.game.intake.rollers):
            self._stage_angles[i] += stage.rpm * math.tau / 60.0 * dt
        st = self.sim.state
        if not self.trail or math.hypot(st.x - self.trail[-1][0],
                                        st.y - self.trail[-1][1]) > 0.02:
            self.trail.append([st.x, st.y])
            self.trail = self.trail[-400:]
        return self.wall_contact

    def snapshot(self, metro_pose, telemetry=None, commands=None):
        """Return detached native-viewer data without moving physics or sensors."""
        if not isinstance(metro_pose, (list, tuple)) or len(metro_pose) != 3:
            raise ValueError("metro_pose must be (x_in, y_in, heading_deg)")
        mx, my, mh = (_finite(v, "metro_pose") for v in metro_pose)
        sim, game = self.sim, self.game
        st, ch, bat, dyn = sim.state, sim.chassis, sim.battery, sim.dynamics
        ox, oy, ot = my * INCH, -mx * INCH, -math.radians(mh)
        left = [u for u in ch.left_units() if u.driven]
        right = [u for u in ch.right_units() if u.driven]
        motors = [{"name": m.name, "rpm": m.velocity * 30.0 / math.pi,
                   "amps": m.current, "temp": m.temperature,
                   "volts": m.applied_voltage, "limited": m.current_limited,
                   "derate": m.derate} for m in sim.motors]
        wheels = [{"name": u.name, "x": u.x, "y": u.y, "slip": u.slip_measure,
                   "fz": u.fz, "fx": u.fx, "fy": u.fy,
                   "rpm": u.omega * 30.0 / math.pi, "angle_rad": u.angle}
                  for u in ch.wheel_units]
        warnings = list(_WARNINGS)
        if self.reverse_transport_unsupported:
            warnings.append("Current reverse-only output has no modeled powered transport; held blocks are passive.")
        if self.outputs["scraper"] or self.outputs["wing"]:
            warnings.append("An unsupported scraper/wing is currently deployed in commands; no collision shape has been added.")
        left_volts = _mean(m.applied_voltage for u in left for m in u.motors)
        right_volts = _mean(m.applied_voltage for u in right for m in u.motors)
        cmd = {"fwd": (left_volts + right_volts) / 2,
               "turn": (right_volts - left_volts) / 2, "stop": ""}
        if commands is not None:
            if not isinstance(commands, dict):
                raise ValueError("commands must be a native cmd dictionary")
            cmd.update(_safe_json(commands))
        physical = {
            "mass_kg": ch.mass, "mass_lb": ch.mass / LB,
            "length_in": ch.length / INCH, "width_in": ch.width / INCH,
            "track_width_in": ch.track_width / INCH,
            "inertia_z_kg_m2": ch.inertia_z, "surface_mu": ch.surface_mu,
            "drive_motors": len(sim.motors),
            "cartridges": [m.cartridge for m in sim.motors],
            "drive_wheels": [{"name": u.name, "diameter_in": 2 * u.radius / INCH,
                              "motor_turns_per_wheel_turn": u.gear_ratio,
                              "friction_coefficients": {
                                  key: getattr(u.wheel.tire, key) * ch.surface_mu
                                  for key in ("mu_long_static", "mu_long_slide",
                                              "mu_lat_static", "mu_lat_slide")},
                              "physical_inches_per_motor_turn": math.tau * u.radius / INCH / u.gear_ratio}
                             for u in ch.driven_units()],
        }
        result = {
            "t": sim.t, "paused": False, "preset": ch.name, "alliance": self.alliance,
            "robot": {"x": st.x, "y": st.y, "theta": st.theta,
                      "length": ch.length, "width": ch.width,
                      "speed_in": math.hypot(st.u, st.v) / INCH,
                      "u_in": st.u / INCH, "v_in": st.v / INCH,
                      "yaw_dps": math.degrees(st.r),
                      "accel_g": math.hypot(st.ax, st.ay) / G},
            "odom": {"x": ox, "y": oy, "theta": ot,
                     "error_in": math.hypot(ox - st.x, oy - st.y) / INCH},
            "battery": {"volts": bat.voltage, "amps": bat.current, "soc": bat.soc,
                        "watts": bat.power_out, "brownout": bat.brownout},
            "power": {"electrical": bat.power_out, "slip": dyn.wheel_slip_power,
                      "rolling": dyn.rolling_loss_power, "drag": dyn.drag_power},
            "spin": {"left": _mean(u.omega for u in left) * 30.0 / math.pi,
                     "right": _mean(u.omega for u in right) * 30.0 / math.pi,
                     "intake": 1.0 if game.rollers_on else 0.0},
            "cmd": cmd, "motors": motors, "wheels": wheels,
            "blocks": [{"x": b.x, "y": b.y, "z": b.z, "yaw": b.yaw,
                        "c": b.colour, "s": b.state,
                        "q": [b.qx, b.qy, b.qz, b.qw]} for b in game.blocks],
            "trail": deepcopy(self.trail), "game": game.snapshot(),
            "warnings": warnings,
            "metro": {
                "actual_pose": {"x_in": -st.y / INCH, "y_in": st.x / INCH,
                                "heading_deg": -math.degrees(st.theta)},
                "odometry_pose": {"x_in": mx, "y_in": my, "heading_deg": mh},
                "telemetry": _safe_json(telemetry), "outputs": dict(self.outputs),
                "wheel_angles_rad": {"left": _mean(u.angle for u in left),
                                     "right": _mean(u.angle for u in right)},
                "native_stage_angles_rad": list(self._stage_angles),
                "wall_contact": self.wall_contact,
                "seed": self.seed, "preloads": list(self.preload_colours),
                "physical_configuration": physical, "assumptions": deepcopy(self.assumptions),
                "fidelity": {
                    "scope": "native_push_back_with_approximate_metro_mechanisms",
                    "warnings": warnings,
                    "unsupported": ["scraper_collision", "wing_collision", "pneumatic_dynamics",
                                    "custom_intake_geometry", "intake_motor_electrical_load",
                                    "optical_distance_mechanism_sensors"],
                    "route_mapping": "hood+positive top: long; flap+negative top: center with upper surface sign reversed; reverse: lower with both surface signs reversed; PRIME backs down long with gate closed. Held-route remapping is approximate",
                    "stage_model": "native lower four / upper two; signed command-scaled nominal RPM; zero commands supply zero traction",
                    "reverse_transport_unsupported_now": self.reverse_transport_unsupported,
                    "control_pose_source": "supplied_actual_metro_cpp_odometry",
                },
            },
        }
        return _safe_json(result)
