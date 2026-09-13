"""Independent mechanics checks for the sibling vexsim package.

Run with PYTHONDONTWRITEBYTECODE=1 python3 tests/vexsim/physics_mechanical_audit.py.
No simulator source is modified. Failing assertions are actual audit findings.
The --diagnostics option prints reproducible numerical experiments as JSON.
"""

import argparse
import copy
import json
import math
from pathlib import Path
import sys
import unittest


def import_simulator(path):
    sys.path.insert(0, str(Path(path).resolve()))
    global wheels, presets, units, Chassis, WheelUnit, Dynamics, normal_loads
    global Simulator, SimConfig, InfiniteBattery
    from vexsim import wheels, presets, units
    from vexsim.chassis import Chassis, WheelUnit
    from vexsim.dynamics import Dynamics, normal_loads
    from vexsim.sim import Simulator, SimConfig
    from vexsim.battery import InfiniteBattery


def positions(ch):
    return [(u.x - ch.cg_x, u.y - ch.cg_y) for u in ch.wheel_units]


def free_body(**kwargs):
    return Chassis(mass=10, inertia_z=1, cg_height=0,
                   bearing_drag=0, yaw_drag=0, aero_drag=0, **kwargs)


class FixedTorqueMotor:
    """A signed shaft power source, independent of vexsim's motor model."""

    output_inertia = 0.003
    free_speed = 100
    velocity = 0
    position = 0

    def __init__(self, torque):
        self.torque = torque

    def output_torque(self):
        return self.torque


class MechanicalAudit(unittest.TestCase):
    def assert_balance(self, ch, ax, ay, p=None):
        p = positions(ch) if p is None else p
        z = normal_loads(ch, ax, ay, p)
        self.assertTrue(all(v >= 0 for v in z))
        self.assertAlmostEqual(sum(z), ch.mass * ch.gravity, places=9)
        self.assertAlmostEqual(sum(x * v for (x, y), v in zip(p, z)),
                               -ch.cg_height * ch.mass * ax, places=9)
        self.assertAlmostEqual(sum(y * v for (x, y), v in zip(p, z)),
                               -ch.cg_height * ch.mass * ay, places=9)

    def test_unit_dimensions_and_round_trips(self):
        self.assertAlmostEqual(units.FOOT, 12 * units.INCH)
        self.assertAlmostEqual(units.TILE, 24 * units.INCH)
        self.assertAlmostEqual(units.IN_LB, units.INCH * units.LB * units.G)
        for x in (-1000, -1, 0, 1, 1000):
            self.assertAlmostEqual(units.to_inches(units.inches(x)), x)
            self.assertAlmostEqual(units.to_rpm(units.rpm(x)), x)
            self.assertAlmostEqual(units.to_deg(units.deg(x)), x)
        self.assertEqual(units.wrap_pi(-math.pi), math.pi)

    def test_gearing_reflected_kinetic_energy(self):
        motor = FixedTorqueMotor(1)
        u = WheelUnit(wheels.omni(), 0, 0, motors=[motor], gear_ratio=3)
        u.omega, u.angle = 7, 11
        u.sync_motors()
        self.assertEqual(motor.velocity, 21)
        self.assertEqual(motor.position, 33)
        axle_energy = 0.5 * u.effective_inertia * u.omega ** 2
        component_energy = (0.5 * u.wheel.inertia * u.omega ** 2
                            + 0.5 * motor.output_inertia * motor.velocity ** 2)
        self.assertAlmostEqual(axle_energy, component_energy)

    def test_external_gearing_forward_power_is_dissipative(self):
        m = FixedTorqueMotor(2)
        u = WheelUnit(wheels.omni(), 0, 0, motors=[m], gear_ratio=3, efficiency=.8)
        u.omega = 10
        u.sync_motors()
        upstream = m.torque * m.velocity
        downstream = u.motor_torque() * u.omega
        self.assertLessEqual(downstream, upstream)
        self.assertAlmostEqual(downstream, .8 * upstream)

    def test_external_gearing_backdrive_does_not_create_power(self):
        m = FixedTorqueMotor(-2)
        u = WheelUnit(wheels.omni(), 0, 0, motors=[m], gear_ratio=3, efficiency=.8)
        u.omega = 10
        u.sync_motors()
        upstream_absorption = -m.torque * m.velocity
        wheel_supply = -u.motor_torque() * u.omega
        self.assertLessEqual(upstream_absorption, wheel_supply,
                             f"upstream absorbs {upstream_absorption} W from "
                             f"only {wheel_supply} W at the wheel")

    def test_all_presets_static_force_and_moment_balance(self):
        for name in presets.ALL:
            with self.subTest(name=name):
                self.assert_balance(presets.get(name), 0, 0)

    def test_all_presets_moderate_acceleration_balance(self):
        for name in presets.ALL:
            with self.subTest(name=name):
                self.assert_balance(presets.get(name), .5, .4)

    def test_two_supports_keep_the_feasible_roll_balance(self):
        ch = Chassis(mass=10, cg_height=.1, wheel_units=[
            WheelUnit(wheels.omni(), 0, -.2), WheelUnit(wheels.omni(), 0, .2)])
        # The pitch constraint is redundant, not inconsistent. Required loads
        # are 54.03325 and 44.03325 N, both positive.
        self.assert_balance(ch, 0, 2)

    def test_four_supports_feasible_corner_load_balance(self):
        ch = Chassis(mass=10, cg_height=.1, wheel_units=[
            WheelUnit(wheels.omni(), x, y)
            for x in (-.2, .2) for y in (-.2, .2)])
        # One contact is lifted while a positive three-contact solution exists.
        self.assert_balance(ch, 19, 19)

    def test_arbitrary_feasible_support_load_balance(self):
        p = [(-.19428983761579338, -.1616212224661262),
             (-.06533825578386507, -.1470973974740411),
             (-.15031808804978392, .1543019552996877),
             (.10802684297858606, -.17577880650135205)]
        # These independently supplied positive loads witness feasibility.
        witness = [95.65050271085562, .18912182934328534,
                   .42667746916470706, 1.8001979906363983]
        ch = Chassis(mass=10, cg_height=.1, wheel_units=[
            WheelUnit(wheels.omni(), x, y) for x, y in p])
        ax = -sum(x * f for (x, y), f in zip(p, witness))
        ay = -sum(y * f for (x, y), f in zip(p, witness))
        self.assert_balance(ch, ax, ay, p)

    def test_steady_tires_are_passive_and_respect_axis_limits(self):
        specs = [factory() for factory in wheels.PRESETS.values()]
        specs += [wheels.tracking(), wheels.mecanum(roller_sign=-1)]
        specs += [wheels.flex(shore=s) for s in ('30A', '35A', '45A', '60A')]
        for spec in specs:
            for k in (-10, -.3, -.02, 0, .02, .3, 10):
                for a in (-10, -.3, -.02, 0, .02, .3, 10):
                    fx, fy = wheels.tire_forces(spec.tire, 50, k, a, .7)
                    self.assertLessEqual(-fx * k + fy * a, 1e-12)
                    self.assertLessEqual(abs(fx), 50 * .7 * spec.tire.mu_long_static)
                    self.assertLessEqual(abs(fy), 50 * .7 * spec.tire.mu_lat_static)
                    rev = wheels.tire_forces(spec.tire, 50, -k, -a, .7)
                    self.assertAlmostEqual(rev[0], -fx)
                    self.assertAlmostEqual(rev[1], -fy)

    def test_coupled_traction_obeys_friction_ellipse(self):
        tire = wheels.traction().tire
        for k in (-10, -.15, -.02, .01, .1, 10):
            for a in (-5, -.2, 0, .1, 5):
                fx, fy = wheels.tire_forces(tire, 30, k, a)
                ellipse = (fx / (30 * tire.mu_long_static)) ** 2
                ellipse += (fy / (30 * tire.mu_lat_static)) ** 2
                self.assertLessEqual(ellipse, 1 + 1e-12)

    def test_contact_velocity_for_steered_omni_and_mecanum(self):
        for gamma in (0, math.pi / 2, math.pi / 4, -math.pi / 4):
            for beta in (-.4, 0, .7):
                spec = wheels.omni()
                spec.roller_angle = gamma
                wheel = WheelUnit(spec, .2, -.1, steer_angle=beta)
                ch = free_body(wheel_units=[wheel])
                d = Dynamics(ch)
                d.state.u, d.state.v, d.state.r = .8, -.3, 1.2
                local_x, local_y = .8 - 1.2 * -.1, -.3 + 1.2 * .2
                wx = math.cos(beta) * local_x + math.sin(beta) * local_y
                wy = -math.sin(beta) * local_x + math.cos(beta) * local_y
                # Modern Robotics: r*omega = wx + tan(gamma_MR)*wy.
                # Its gamma is zero for omni; vexsim stores pi/2 for omni.
                roller_projection = 0 if gamma in (0, math.pi / 2) else math.copysign(1, gamma)
                wheel.omega = (wx + roller_projection * wy) / wheel.radius
                d.step(1e-8)
                self.assertAlmostEqual(wheel.kappa_ss, 0, places=12)

    def test_free_body_acceleration_and_constant_force_translation(self):
        ch = free_body()
        d = Dynamics(ch)
        d.ext_fx, d.ext_fy, d.ext_mz = 20, -30, 0
        for i in range(1000):
            d.step(.001)
        self.assertAlmostEqual(d.state.u, 2, places=10)
        self.assertAlmostEqual(d.state.v, -3, places=10)
        self.assertLess(abs(d.state.x - 1), .0011)
        self.assertLess(abs(d.state.y + 1.5), .0016)

    def test_world_rotation_covariance(self):
        a = Dynamics(presets.speed_base())
        b = Dynamics(copy.deepcopy(a.chassis))
        phi = 1.1
        a.state.u = b.state.u = 1
        a.state.v = b.state.v = .3
        a.state.r = b.state.r = -.2
        b.state.theta = phi
        for i in range(1000):
            a.step(.001)
            b.step(.001)
        self.assertAlmostEqual(b.state.x, math.cos(phi) * a.state.x - math.sin(phi) * a.state.y)
        self.assertAlmostEqual(b.state.y, math.sin(phi) * a.state.x + math.cos(phi) * a.state.y)
        self.assertAlmostEqual(b.state.theta - a.state.theta, phi)

    def test_force_free_rotating_body_has_at_most_one_percent_energy_drift(self):
        # No tire, drag, motor, external force, or initial elastic slip energy.
        # The world velocity should remain exactly constant while heading turns.
        d = Dynamics(free_body())
        d.state.u, d.state.r = 1, 5
        initial_translation_energy = 5
        for i in range(1000):
            d.step(.001)
        final_translation_energy = .5 * d.chassis.mass * d.state.speed ** 2
        relative_error = final_translation_energy / initial_translation_energy - 1
        self.assertLessEqual(abs(relative_error), .01)

    def test_unpowered_coasting_cannot_exceed_initial_kinetic_energy(self):
        for factory in (wheels.omni, wheels.traction, wheels.flex,
                        wheels.tracking, wheels.mecanum):
            with self.subTest(wheel=factory.__name__):
                ch = free_body(wheel_units=[WheelUnit(factory(), x, y)
                               for x in (-.15, .15) for y in (-.15, .15)])
                d = Dynamics(ch)
                d.state.u, d.state.v, d.state.r = .8, .3, .5
                for u in ch.wheel_units:
                    u.omega = .8 / u.radius
                initial = .5 * ch.mass * (.8 ** 2 + .3 ** 2)
                initial += .5 * ch.inertia_z * .5 ** 2
                initial += sum(.5 * u.effective_inertia * u.omega ** 2
                               for u in ch.wheel_units)
                for i in range(1000):
                    d.step(.001)
                    self.assertLessEqual(d.kinetic_energy, initial + 1e-9)

    def test_reported_loss_accounts_for_isolated_yaw_scrubbing(self):
        ch = free_body(surface_mu=0, wheel_units=[
            WheelUnit(wheels.omni(), x, y)
            for x in (-.15, .15) for y in (-.15, .15)])
        d = Dynamics(ch)
        d.state.r = 1
        initial = .5 * ch.inertia_z
        dt = 1e-7
        d.step(dt)
        observed_loss = (initial - d.kinetic_energy) / dt
        reported_loss = d.drag_power + d.wheel_slip_power + d.rolling_loss_power
        self.assertAlmostEqual(observed_loss, reported_loss, places=5)

    def test_high_speed_arc_endpoint_is_stable_across_timestep_refinement(self):
        coarse, medium, fine = [arc_run(dt) for dt in (.001, .0005, .0002)]
        # A 2 mm / quarter-degree audit budget after 3 s is deliberately much
        # smaller than the multi-inch endpoint and odometry errors under study.
        for row in (coarse, medium):
            self.assertLess(math.hypot(row['pose'][0] - fine['pose'][0],
                                       row['pose'][1] - fine['pose'][1]), .002)
            self.assertLess(abs(row['pose'][2] - fine['pose'][2]), math.radians(.25))


def arc_run(dt, left=12, right=8, duration=3, *, ideal_battery=False,
            relaxation=None, heading=0):
    ch = presets.speed_base()
    if relaxation is not None:
        for u in ch.wheel_units:
            u.wheel.tire.relaxation_length = relaxation
    sim = Simulator(ch, battery=InfiniteBattery() if ideal_battery else None,
                    config=SimConfig(dt=dt, log_every=0))
    sim.state.theta = heading
    for group, voltage in ((ch.left_units(), left), (ch.right_units(), right)):
        for u in group:
            for m in u.motors:
                m.set_voltage(voltage)
    lat_integral = max_lat = positive_contact_work = signed_contact_loss = 0
    min_total_loss = 0
    for i in range(round(duration / dt)):
        # Save pre-step velocities: reported forces act at these velocities.
        old = [(u.omega, u.radius) for u in ch.wheel_units]
        sim.step()
        signed = sum(u.fx * (om * rad - u.vx) - u.fy * u.vy
                     for u, (om, rad) in zip(ch.wheel_units, old))
        signed_contact_loss += signed * dt
        positive_contact_work += max(0, -signed) * dt
        min_total_loss = min(min_total_loss, signed)
        max_lat = max(max_lat, abs(sim.state.v))
        lat_integral += abs(sim.state.v) * dt
    s = sim.state
    return dict(dt=dt, voltage=[left, right], duration=duration,
                pose=[s.x, s.y, s.theta], body_velocity=[s.u, s.v, s.r],
                max_lateral_speed_mps=max_lat, abs_lateral_travel_m=lat_integral,
                odom_error_m=math.hypot(sim.odometry.x - s.x, sim.odometry.y - s.y),
                signed_contact_loss_J=signed_contact_loss,
                logged_contact_loss_J=sim.energy_slip,
                positive_net_contact_work_J=positive_contact_work,
                minimum_signed_contact_loss_W=min_total_loss,
                kinetic_energy_J=sim.dynamics.kinetic_energy,
                battery_energy_J=sim.energy_battery)


def diagnostics():
    result = {}
    ch = presets.odom_bot()
    loads = normal_loads(ch, 0, 0, positions(ch))
    result['tracking_preload'] = dict(weight_N=ch.weight, loads=[
        dict(name=u.name, preload_N=u.preload, normal_N=f)
        for u, f in zip(ch.wheel_units, loads)])
    result['arcs'] = []
    for pair in ((12, 8), (8, 12), (6, 3), (12, 12)):
        for dt in (.001, .0005, .0002):
            result['arcs'].append(arc_run(dt, *pair))
    result['arc_detailed_convergence'] = [arc_run(dt) for dt in (.0001, .00005)]
    result['arcs_without_tire_lag'] = [
        arc_run(dt, relaxation=.0001) for dt in (.001, .0005, .0002)]
    result['arcs_ideal_battery'] = [
        arc_run(dt, ideal_battery=True) for dt in (.001, .0002)]
    result['force_free_rotation'] = []
    for dt in (.001, .0005, .0002):
        d = Dynamics(free_body())
        d.state.u, d.state.r = 1, 5
        for i in range(round(1 / dt)):
            d.step(dt)
        result['force_free_rotation'].append(dict(
            dt=dt, pose=[d.state.x, d.state.y, d.state.theta],
            world_velocity=d.state.world_velocity(),
            translational_energy_ratio=d.state.speed ** 2))
    # An isolated existing yaw loss must appear somewhere in the loss outputs.
    c = presets.four_motor_200()
    c.bearing_drag = c.yaw_drag = c.aero_drag = c.surface_mu = 0
    d = Dynamics(c)
    d.state.r = 1
    before = .5 * c.inertia_z
    d.step(1e-6)
    result['scrub_loss_accounting'] = dict(
        mechanical_energy_decrease_W=(before-d.kinetic_energy)/1e-6,
        sum_reported_losses_W=d.drag_power+d.wheel_slip_power+d.rolling_loss_power,
        yaw_moment_Nm=d.total_mz)
    print(json.dumps(result, indent=2))


if __name__ == '__main__':
    parser = argparse.ArgumentParser()
    parser.add_argument('--vexsim', default=str(Path(__file__).resolve().parents[2].parent / 'vexsim'))
    parser.add_argument('--diagnostics', action='store_true')
    args, remaining = parser.parse_known_args()
    import_simulator(args.vexsim)
    if args.diagnostics:
        diagnostics()
    else:
        unittest.main(argv=[sys.argv[0], *remaining], verbosity=2)
