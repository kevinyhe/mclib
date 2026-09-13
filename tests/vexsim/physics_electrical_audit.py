"""Independent electrical/thermal audit of the unmodified sibling vexsim.

Run from mclib with:
    PYTHONDONTWRITEBYTECODE=1 python3 tests/vexsim/physics_electrical_audit.py

Override the sibling location with VEXSIM_PATH. Failures are actual audit
failures, never expected failures. No hardware measurement is implied.
The tests use terminal energy conservation, closed-form RL/thermal solutions,
and manufacturer limits, rather than trusting vexsim's reporting properties.

Manufacturer references consulted 2026-09-08:
https://kb.vex.com/hc/en-us/articles/360044325872-Understanding-V5-Smart-Motor-11W-Performance
https://kb.vex.com/hc/en-us/articles/10002101702932-Understanding-V5-Smart-Motor-5-5W-Performance
https://www.vexforum.com/t/v5-motor-current-limit-vs-temperature/107172
https://api.vex.com/v5/home/cpp/Motors_and_MotorControllers/motor_and_motor_group.html
https://www.vexrobotics.com/276-4811.html
https://projectpartners.pltw.org/Training3/story_content/external_files/VEX_2_Wire_Motor_393.pdf
The last document is VEX's manufacturer datasheet hosted by PLTW.
"""

import json
import math
import os
from pathlib import Path
import sys
import unittest


sys.dont_write_bytecode = True
sys.path.insert(0, os.environ.get(
    "VEXSIM_PATH", str(Path(__file__).resolve().parents[3] / "vexsim")))

from vexsim import Battery, InfiniteBattery, Motor, Simulator, presets
from vexsim.motor import BrakeMode, cortex_393, v5_11w, v5_5_5w
from vexsim.sim import SimConfig


MODEL_CASES = [
    (v5_11w, "red"), (v5_11w, "green"), (v5_11w, "blue"),
    (v5_11w, "direct"), (v5_5_5w, "green"),
    (cortex_393, "393_torque"), (cortex_393, "393_speed"),
]


def solve_at(motor, voltage, speed, dt=0.001, bus=12.0):
    motor.velocity = speed
    motor.set_voltage(voltage)
    draw = motor.electrical(bus, dt)
    motor.output_torque()
    return draw


def equilibrium_speed(motor, voltage, load=0.0):
    """Find the actual torque-zero/load crossing, not Motor.free_speed."""
    low, high = 0.0, 2000.0 / motor.ratio
    for _ in range(70):
        middle = 0.5 * (low + high)
        solve_at(motor, voltage, middle, bus=voltage)
        if motor.torque > load:
            low = middle
        else:
            high = middle
    return 0.5 * (low + high)


def curve_metrics(factory, cartridge, voltage):
    motor = Motor(spec=factory(), cartridge=cartridge)
    solve_at(motor, voltage, 0.0, bus=voltage)
    stall_torque, stall_current = motor.torque, motor.current
    free = equilibrium_speed(motor, voltage)
    peak_power, peak_speed = 0.0, 0.0
    for j in range(2001):
        speed = free * j / 2000
        solve_at(motor, voltage, speed, bus=voltage)
        if motor.power_mechanical > peak_power:
            peak_power, peak_speed = motor.power_mechanical, speed
    return {
        "name": motor.spec.name, "cartridge": cartridge, "voltage_V": voltage,
        "free_rpm": free * 30 / math.pi, "stall_torque_Nm": stall_torque,
        "stall_current_A": stall_current, "peak_power_W": peak_power,
        "peak_power_rpm": peak_speed * 30 / math.pi,
    }


class ElectricalPhysicsAudit(unittest.TestCase):
    def test_dc_winding_equation_all_models_and_cartridges(self):
        for factory, cartridge in MODEL_CASES:
            for fraction in (-1.2, -0.5, 0.0, 0.5, 1.2):
                for voltage in (-12.0, -3.0, 0.0, 3.0, 12.0):
                    motor = Motor(spec=factory(), cartridge=cartridge)
                    solve_at(motor, voltage, fraction * motor.free_speed)
                    with self.subTest(model=motor.spec.name, cartridge=cartridge,
                                      speed=fraction, voltage=voltage):
                        emf = motor.spec.ke * motor.velocity * motor.ratio
                        self.assertAlmostEqual(
                            motor.applied_voltage,
                            motor.current * motor.spec.resistance + emf, places=10)
                        self.assertLessEqual(abs(motor.current), motor.current_limit)

    def test_pwm_power_transfer_all_models_and_cartridges(self):
        for factory, cartridge in MODEL_CASES:
            for fraction in (-0.5, 0.0, 0.5, 1.2):
                for voltage in (-12.0, -3.0, 0.0, 3.0, 12.0):
                    motor = Motor(spec=factory(), cartridge=cartridge)
                    draw = solve_at(motor, voltage, fraction * motor.free_speed)
                    with self.subTest(model=motor.spec.name, cartridge=cartridge,
                                      speed=fraction, voltage=voltage):
                        self.assertAlmostEqual(
                            12 * draw, motor.power_electrical, places=10)

    def test_torque_direction_symmetry_all_models(self):
        for factory, cartridge in MODEL_CASES:
            forward = Motor(spec=factory(), cartridge=cartridge)
            reverse = Motor(spec=factory(), cartridge=cartridge)
            solve_at(forward, 9.0, 0.3 * forward.free_speed)
            solve_at(reverse, -9.0, -0.3 * reverse.free_speed)
            with self.subTest(model=forward.spec.name, cartridge=cartridge):
                self.assertAlmostEqual(forward.torque, -reverse.torque, places=10)
                self.assertAlmostEqual(
                    forward.power_electrical, reverse.power_electrical, places=10)

    def test_brake_cannot_create_heat_all_models(self):
        for factory, cartridge in MODEL_CASES:
            motor = Motor(spec=factory(), cartridge=cartridge)
            motor.velocity = 0.5 * motor.free_speed
            motor.stop(BrakeMode.BRAKE)
            draw = motor.electrical(12.0, 0.001)
            motor.output_torque()
            with self.subTest(model=motor.spec.name, cartridge=cartridge):
                # No inductance and fixed speed: input must fund ALL heat.
                self.assertGreaterEqual(
                    12 * draw - motor.power_mechanical + 1e-10,
                    motor.power_copper + motor.power_friction,
                    "Shorted brake dissipates more heat than supplied at its shaft")

    def test_motoring_ledger_accounts_for_gearbox_heat(self):
        motor = Motor()
        solve_at(motor, 12.0, 0.5 * motor.free_speed)
        self.assertAlmostEqual(
            motor.power_electrical - motor.power_mechanical,
            motor.power_copper + motor.power_friction, places=9,
            msg="Steady motor input minus output must appear as motor heat")

    def test_current_limited_brake_retains_bus_power_balance(self):
        motor = Motor()
        motor.velocity = motor.free_speed
        motor.stop(BrakeMode.BRAKE)
        draw = motor.electrical(12.0, 0.001)
        self.assertAlmostEqual(12 * draw, motor.power_electrical, places=9)

    def test_active_bridge_cannot_output_more_than_supply_voltage(self):
        motor = Motor()
        solve_at(motor, 0.0, 3.0 * motor.free_speed)
        self.assertLessEqual(abs(motor.applied_voltage), 12.0 + 1e-10)

    def test_rl_step_and_composition_without_current_clipping(self):
        for factory in (v5_11w, v5_5_5w, cortex_393):
            motor = Motor(spec=factory(), electrical_dynamics=True)
            split = Motor(spec=factory(), electrical_dynamics=True)
            voltage, duration = 3.0, 0.0005
            motor.set_voltage(voltage)
            split.set_voltage(voltage)
            motor.electrical(12.0, duration)
            for _ in range(10):
                split.electrical(12.0, duration / 10)
            spec = motor.spec
            exact = voltage / spec.resistance * (
                1 - math.exp(-spec.resistance * duration / spec.inductance))
            with self.subTest(model=spec.name):
                self.assertAlmostEqual(motor.current, exact, places=12)
                self.assertAlmostEqual(motor.current, split.current, places=12)

    def test_simulator_electrical_retries_advance_inductance_once(self):
        sim = Simulator(presets.four_motor_200(), InfiniteBattery(),
                        SimConfig(log_every=0))
        for motor in sim.motors:
            motor.electrical_dynamics = True
            motor.set_voltage(3.0)
        sim._solve_electrical(0.0005)
        spec = sim.motors[0].spec
        exact = 3.0 / spec.resistance * (
            1 - math.exp(-spec.resistance * 0.0005 / spec.inductance))
        for motor in sim.motors:
            self.assertAlmostEqual(motor.current, exact, places=12)

    def test_thermal_rc_matches_closed_form(self):
        for factory in (v5_11w, v5_5_5w, cortex_393):
            motor = Motor(spec=factory())
            motor.power_copper = 3.0
            motor.power_friction = 1.0
            duration, dt = 10.0, 0.001
            for _ in range(round(duration / dt)):
                motor.thermal_step(dt)
            spec = motor.spec
            exact = motor.ambient_temperature + 4 * spec.thermal_resistance * (
                1 - math.exp(-duration / (
                    spec.thermal_resistance * spec.thermal_capacitance)))
            with self.subTest(model=spec.name):
                self.assertAlmostEqual(motor.temperature, exact, delta=0.0001)

    def test_v5_11w_temperature_derating_matches_manufacturer(self):
        # Cited temperature/fraction schedule is established for the 11W motor.
        # The 5.5W page describes derating but does not publish this exact table.
        for temperature, fraction in (
                (54.0, 1.0), (55.0, 0.5), (60.0, 0.25),
                (65.0, 0.125), (70.0, 0.0), (75.0, 0.0)):
            motor = Motor(spec=v5_11w())
            motor.temperature = temperature
            solve_at(motor, 12.0, 0.0)
            with self.subTest(model=motor.spec.name, temperature=temperature):
                self.assertAlmostEqual(motor.current, 2.5 * fraction, places=9)

    def test_v5_11w_basic_endpoints_on_three_cartridges(self):
        for cartridge, nominal_rpm, nominal_stall in (
                ("red", 100.0, 2.1), ("green", 200.0, 1.05),
                ("blue", 600.0, 0.35)):
            metrics = curve_metrics(v5_11w, cartridge, 12.0)
            with self.subTest(cartridge=cartridge):
                self.assertAlmostEqual(metrics["free_rpm"], nominal_rpm,
                                       delta=0.03 * nominal_rpm)
                self.assertAlmostEqual(metrics["stall_torque_Nm"], nominal_stall,
                                       delta=0.1 * nominal_stall)
                self.assertAlmostEqual(metrics["peak_power_W"], 11.0, delta=0.55)

    def test_v5_11w_retains_nominal_speed_at_thirty_percent_stall_load(self):
        # VEX states nominal RPM persists up to about 35% stall torque,
        # including the RPM limits retained in raw PWM mode. Allow 5% RPM error.
        for cartridge, nominal_rpm, nominal_stall in (
                ("red", 100.0, 2.1), ("green", 200.0, 1.05),
                ("blue", 600.0, 0.35)):
            motor = Motor(cartridge=cartridge)
            speed = equilibrium_speed(motor, 12.0, 0.3 * nominal_stall)
            with self.subTest(cartridge=cartridge):
                self.assertAlmostEqual(speed * 30 / math.pi, nominal_rpm,
                                       delta=0.05 * nominal_rpm)

    def test_v5_5_5w_actual_curve_matches_manufacturer(self):
        metrics = curve_metrics(v5_5_5w, "green", 12.0)
        for key, nominal, tolerance in (
                ("free_rpm", 200.0, 10.0),
                ("stall_torque_Nm", 0.5, 0.05),
                ("peak_power_W", 5.5, 0.55)):
            with self.subTest(metric=key):
                self.assertAlmostEqual(metrics[key], nominal, delta=tolerance)

    def test_393_at_datasheet_voltage_matches_manufacturer(self):
        # Manufacturer explicitly permits 20% unit variation at 7.2 V.
        for cartridge, nominal_rpm, nominal_stall in (
                ("393_torque", 100.0, 1.67), ("393_speed", 160.0, 1.04)):
            metrics = curve_metrics(cortex_393, cartridge, 7.2)
            for key, nominal in (("free_rpm", nominal_rpm),
                                 ("stall_torque_Nm", nominal_stall),
                                 ("stall_current_A", 4.8)):
                with self.subTest(cartridge=cartridge, metric=key):
                    self.assertAlmostEqual(metrics[key], nominal,
                                           delta=0.2 * nominal)

    def test_battery_charge_coulomb_counting_and_power_identity(self):
        battery = Battery(soc=0.75)
        original_soc = battery.soc
        used = 0.0
        for current in (2.0, 10.0, -2.0, 0.0):
            ocv = battery.open_circuit_voltage()
            battery.update(current, 0.1)
            used += current * 0.1 / 3600
            self.assertAlmostEqual(
                ocv * current, battery.power_out + battery.power_loss, places=10)
        self.assertAlmostEqual(battery.charge_used_ah, used, places=12)
        self.assertAlmostEqual(battery.soc, original_soc - used / battery.capacity_ah,
                               places=12)

    def test_infinite_battery_has_no_discharge_or_sag(self):
        battery = InfiniteBattery()
        for current in (0.0, 10.0, -10.0, 100.0):
            battery.update(current, 10.0)
            self.assertEqual(battery.voltage, 12.0)
            self.assertEqual(battery.soc, 1.0)
            self.assertEqual(battery.charge_used_ah, 0.0)
            self.assertEqual(battery.power_out, 12 * current)

    def test_empty_finite_battery_cannot_supply_robot_energy(self):
        sim = Simulator(presets.four_motor_200(), Battery(soc=0.0),
                        SimConfig(log_every=0))
        for motor in sim.motors:
            motor.set_voltage(12.0)
        for _ in range(100):
            sim.step()
        self.assertLessEqual(sim.energy_battery, 1e-10,
                             "An already empty pack supplied new energy")

    def test_brain_current_budget_is_enforced_on_first_step(self):
        sim = Simulator(presets.six_motor_200(),
                        config=SimConfig(brain_current_limit=4.0, log_every=0))
        for motor in sim.motors:
            motor.set_voltage(12.0)
        sim._solve_electrical(0.001)
        self.assertLessEqual(sim.battery.current, 4.0 + 0.01)

    def test_zero_brain_current_budget_disables_power(self):
        sim = Simulator(presets.six_motor_200(),
                        config=SimConfig(brain_current_limit=0.0, log_every=0))
        for motor in sim.motors:
            motor.set_voltage(12.0)
        for _ in range(100):
            sim._solve_electrical(0.001)
        self.assertLessEqual(sim.battery.current, 1e-10)

    def test_default_bus_solution_matches_independent_quadratic(self):
        sim = Simulator(presets.six_motor_200(), config=SimConfig(log_every=0))
        for motor in sim.motors:
            motor.set_voltage(12.0)
        # At zero speed every winding is current limited: fixed resistor power.
        # Use the selected model's declared resistance/current limit: these are
        # fitted parameters, not manufacturer measurements. The independently
        # solved source/load quadratic and its physical tolerance are unchanged.
        power = sum(m.current_limit ** 2 * m.spec.resistance for m in sim.motors)
        ocv, resistance = 14.4, 0.035
        exact = 0.5 * (ocv + math.sqrt(ocv ** 2 - 4 * resistance * power))
        sim._solve_electrical(0.001)
        self.assertAlmostEqual(sim.battery.voltage, exact, delta=0.00001)
        self.assertAlmostEqual(sim.battery.power_out, power, delta=0.001)

    def test_internal_motor_controller_uses_ten_millisecond_period(self):
        sim = Simulator(presets.four_motor_200(), InfiniteBattery(),
                        SimConfig(log_every=0))
        calls = []
        motor = sim.motors[0]
        motor.set_velocity(10.0)
        original = motor.control_voltage

        def observed(dt):
            calls.append(dt)
            return original(dt)

        motor.control_voltage = observed
        for _ in range(10):
            sim._solve_electrical(0.001)
        self.assertEqual(len(calls), 1,
                         "VEX specifies one internal PID update per 10 ms")


def evidence():
    curves = [curve_metrics(factory, cartridge, 12.0)
              for factory, cartridge in MODEL_CASES]
    curves += [curve_metrics(cortex_393, cartridge, 7.2)
               for cartridge in ("393_torque", "393_speed")]
    motor = Motor()
    motor.velocity = 0.5 * motor.free_speed
    motor.stop(BrakeMode.BRAKE)
    draw = motor.electrical(12.0, 0.001)
    motor.output_torque()
    brake = {
        "bus_power_W": 12 * draw, "shaft_power_out_W": motor.power_mechanical,
        "copper_heat_W": motor.power_copper,
        "friction_heat_W": motor.power_friction,
        "energy_residual_W": 12 * draw - motor.power_mechanical
        - motor.power_copper - motor.power_friction,
    }
    return {"motor_curves": curves, "green_motor_half_speed_brake": brake}


if __name__ == "__main__":
    print(json.dumps(evidence(), indent=2), flush=True)
    unittest.main(verbosity=2)
