# Metro mass and used-tile friction

Updated September 9, 2026.

The Metro runner now defaults to the user's approximate **14 lb (6.35029318 kg)**
robot mass, replacing its 16.5 lb assumption. The native chassis constructor
derives yaw inertia from this mass: `1.25 * mass * (length² + width²) / 12`.
With the existing 15 × 12.5 inch body, this is approximately 0.162705 kg·m²,
15.15% below the previous estimate. The mass distribution remains unmeasured.
The native solver uses the new mass for acceleration, normal loads and contacts.

## What the friction research supports

Friction describes a wheel–surface pair, not the mat alone. No verified
coefficient for this robot's 3.25-inch omni wheels on its particular used mats
was found.

- A [2019 first-hand pull test on old VEX tiles](https://www.vexforum.com/t/omni-wheel-vs-mecanum/48151/13)
  used locked wheels, a weighted base and luggage scales. Its author explicitly
  cautions about old wheels/tiles, scale accuracy and rounding. The text reports
  about 1.3 forward for 4-inch omnis, and says 3.25-inch omnis behaved differently.
  These effective pull measurements combine contact/deformation effects; they
  do not identify separate peak and fully sliding tire coefficients for our model.
- An [earlier first-hand incline test](https://www.vexforum.com/t/anyone-have-friction-coefficient-of-vex-wheels/19261/19)
  reported approximately 0.714 for four 4-inch omnis on foam. This differs in
  apparatus, wheels and surface condition; 0.714–1.3 is not a calibrated range
  for this robot or a general used-mat specification.
- [RECF's field notice](https://kb.roboticseducation.org/hc/en-us/articles/5383535573911-VEX-V5-Field-Event-Partner-ESD-Notice)
  describes old-style and anti-static tiles as functionally equivalent with
  negligible coefficient differences. It supplies no numerical wear correction.

## Current simulation choice

At the user's request, reduce original friction by **14%**, using a **0.86 surface
multiplier** on the native omni tire coefficients. The runner defaults to
`--surface-mu 0.86`; previous runs are preserved as separate recordings. These remain simulation
assumptions, not measured coefficients for the actual mats.

| Dimensionless coefficient | Value |
| --- | ---: |
| Longitudinal peak | 1.05 |
| Longitudinal fully sliding | 0.85 |
| Lateral peak | 0.26 |
| Lateral fully sliding | 0.22 |
| Surface multiplier | 0.86 |

Effective coefficients are 0.903/0.731 longitudinal and 0.2236/0.1892 lateral.

`surface_mu` is a multiplier, not the coefficient itself. Each replay frame now
records all four effective tire coefficients per driven wheel; metadata marks
friction as uncalibrated. Wheel diameter, gearing and lateral roller resistance
remain assumptions. Controller gains and odometry are unchanged.

For calibration, use a horizontal force gauge on the actual loaded robot with
drive-wheel rotation locked. Record breakaway force and steady sliding force
separately over several mat locations and both travel directions. Divide force
by weight in matching units: on a 14 lb robot, 14 lbf corresponds to an effective
coefficient of 1.0. Sideways omni motion primarily rolls the rollers, so it needs
its own measurement and should not reuse the longitudinal coefficient.
