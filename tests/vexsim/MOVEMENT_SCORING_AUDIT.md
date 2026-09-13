# Movement and scoring audit — September 9, 2026

Not all movement scenarios pass. The mclib controllers and the external Metro
controllers are separate implementations; a passing mclib test is not a Metro
accuracy guarantee.

## September 10: original targets restored, editable points

The user reverted the early shifts. `POINTS` in
[metro_waypoints.py](metro_waypoints.py) now contains all five original targets.
Default builds compile the original autonomous file directly; editing those
pairs changes only point coordinates in a checked build copy. Goal sliding
remains 150% longer than the former 4x stopping setting (rate 1.6).
See [editing/replay instructions](METRO_GAME_VALIDATION.md). The original goal
alignment and independent motion failures below remain relevant.

## Current movement results

Commands run without changing gains or widening acceptance limits:

```
python3 -B tests/vexsim/run.py --output /tmp/mclib-movement-current
python3 -B tests/vexsim/kinematic_motion_audit.py --output /tmp/mclib-ideal-current
```

The physical matrix passes **70/80**: all 40 safety scenarios and four wall
scenarios pass. Failures are all three boomerangs, all six forward/reverse arcs,
and the six-motor diagonal point move. For six_motor_450, position errors are
2.59 in for the diagonal, 6.76 in for boomerang, and 9.75 in for either arc.

The independent ideal plant passes **28/36**. Failures include two slow-plant
arc deadlines, turn-to-point on the two faster plants, the fastest plant's
left/right turns, and boomerang on both faster plants. The six-motor ideal
boomerang error is 27.16 in. These failures show that slip alone does not explain
all endpoint problems. Raw results and compiled test artifacts remain in the
two temporary directories above. Both commands return failure as intended.

Metro coverage includes its actual full routine, host scheduler/sensor checks,
and compiled signed-arc geometry. It does not independently prove every Metro
movement function accurate. This update does not silently substitute mclib
controllers or feed simulated truth into Metro's steering.

## Why the original Metro replay missed

In the preceding `park-side-odom-offset-fixed` run, the first 500 ms point move
reports local (-6.00, 20.38) in for target (-9, 24), about 4.70 in short. The
routine chains into later motions with its original deadlines while the drive
voltage scale is 0.9. Correcting odometry changes feedback and thus its path;
the waypoints and gains were not retuned along with that correction.

At the first SCORE command, the physical pose is approximately
(-33.12, -34.10) in in Metro coordinates. The long goal's cross-track coordinate
is -46.765 in, so the robot is about **13.6 in sideways from the goal axis**.
The simulation reports no goal in range. A functioning feeder cannot repair
that alignment.

There was also a display bug: the explicit local X/Y reset at 3.83 s restarted
the ghost's field origin. Its displayed error jumped from about 4.6 to 34 in.
The recorder now retains the accumulated odometry origin across this reset;
local C++ coordinates still reset exactly as the routine requests. The ghost
stays in the same field frame without reading truth to estimate its position.
The `actual_pose` diagnostic now uses physical truth, with the supplied
odometry in its separately named field.

## Scoring fixes and limits

Middle scoring uses a positive bottom command and negative top command. On
the native under-roller route, that negative top command must produce positive
travel toward the exit. Previously it fought the feed and stalled blocks.
Low scoring now selects the forward-facing lower route for reverse outputs,
instead of suppressing the motors. PRIME remains on the long route with the
gate closed and physically backs blocks away from the optical sensor.

`python3 -B tests/vexsim/test_metro_scoring.py` reproduces all three original
failures before the fix and passes afterward. Four test methods cover seven
scenarios: aligned middle/low goals, slower three-block feeds from both ends
of both goals, and a misaligned shot that ejects but does not score. Blocks
retain identity; no test or production mapping assigns a successful goal.
Entry, collision and scoring remain the native physics calculation. The
user-selected 4x goal stopping rate is included in these checks.

The constrained intake route and route-change repositioning remain geometric
approximations, not measured Metro mechanism dynamics. The complete autonomous
run still misses the goals with its existing waypoint/deadline configuration;
passing isolated scoring checks does not mean the autonomous scores correctly.
