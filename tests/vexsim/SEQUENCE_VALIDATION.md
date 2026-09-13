# Action-sequence validation — 2026-09-09

## Outcome

The builder executes actions sequentially. **120/120 tested sequences** preserve
call order, stopped command handoffs, the 300 ms hold, pose continuity and the
selected failure policy. These are 20 combinations × three drivetrain presets
× two sensor layouts, running the real C++ routines against vexsim.

There is, however, a **confirmed replay-inspection bug**: selecting an earlier
result at a shared step-boundary timestamp displays the next action's commands
and target while the earlier action remains selected. Skipped-step inspection
also shows a previous action's frame without clearly distinguishing the absence
of a recording. The browser reproduction has **one passing observation and three
failures**, retained as failures. No application code was changed in this
validation pass; approval to implement the replay correction was requested
separately.

Sequential execution does not establish successful motion: **56/96 combinations
without injected failures** meet every motion check. The other **24** cases
deliberately inject timeouts or reject incompatible reverse-arc parameters.
Their action order/failure-policy checks pass, but their failed motion results
remain red. Across all cases, **249/330 executed actions** pass their motion
checks; six additional actions are correctly skipped.

## What made the actions appear non-sequential

The user's three tracker recordings have byte-for-byte identical first-action
traces, including the hold: boomerang alone (`038dfc…`), boomerang → turn
(`77eb01…`), and boomerang → point (`8c88b4…`). Adding a later action does not
alter the first action.

| Recorded action | Motion interval | Following stopped-command hold |
| --- | --- | --- |
| Boomerang, either tracker combination | 0–3.410 s | 3.410–3.710 s |
| Following turn | 3.710–4.860 s | 4.860–5.160 s |
| Following point, separate recording | 3.710–5.950 s | 5.950–6.250 s |

The first boomerang itself turns to +68.495° and later −207.319° before finishing
near −87.890°. That excessive rotation occurs **before** the next action starts.
It is a remaining trajectory defect/limitation, not overlapping execution.

The replay bug is in `builder.js`'s result-row handler and `frameAt()`:

1. Clicking result 1 seeks to its `end_time_ms`, 3.710 seconds.
2. That timestamp contains the final hold frame of step 1 and initial frames of
   step 2. Choosing the last frame at or before the timestamp selects step 2.
3. The UI consequently shows **STEP 02 / MOTION**, **12/−12 V** and **177.89°**
   target error while the selected result and inspector say **STEP 01**.
   The actual step-1 result is stopped, 1.389 inches / 2.110°.

The point combination exhibits the same mismatch, with step 2's **1/1 V** and
26.78-inch target error. Editor selection is also independent of playback: a
selected draft action does not indicate which recorded action is currently
playing. These states need to be visually distinguished; editing a draft must
not silently change recorded data.

In failed drive-encoder run `584d64…`, the boomerang returns at 2.060 seconds,
holds until 2.360, and the point is **skipped** because Stop at first failure is
enabled. No second-action trace exists. Clicking its skipped result currently
shows step 1's final frame alongside the step-2 inspector.

## Combination matrix

Each row below has six cases: `four_motor_200`, `six_motor_450`, `speed_base`,
each with `drive` and `two` sensor layouts. Every row passes **6/6 sequencing
checks**. The right column is separate, unchanged **motion acceptance**.
Most cases explicitly disable stop-on-failure so later actions remain exercised
even when an earlier controller misses its endpoint. `timeout_abort` enables it.
Normal step deadlines remain four seconds; injected deadlines are 50 ms, and the
locked wall-reset deadline is 800 ms. PID defaults, voltages and tolerances are
not altered to obtain passes.

| Combination | All motion checks pass |
| --- | ---: |
| Drive 12 → turn 90° → drive 12 | 6/6 |
| Turn 90° → drive 12 → turn 0° → drive 12 | 6/6 |
| Drive 12 → drive −12 → turn −90° | 6/6 |
| Turn toward point → drive → point | 4/6 |
| Point → drive → turn | 2/6 |
| Lead-zero boomerang → turn → drive | 4/6 |
| User boomerang `(15.5,18.5,−90°)` → turn | 1/6 |
| Same user boomerang → point `(−10.5,25.5)` | 1/6 |
| Reverse boomerang → reverse drive → turn | 2/6 |
| Arc → drive → turn | 1/6 |
| Reverse arc → drive → turn | 2/6 |
| Swing → drive → opposite swing | 6/6 |
| Already-reached turn/point/boomerang → drive | 6/6 |
| Timed-out turn → abort remaining drive | 0/6, injected failure; all six drives skipped |
| Timed-out turn → continue drive → turn | 0/6, injected failure retained |
| Rejected reverse arc → continue drive | 0/6, injected configuration rejection retained |
| Already-reached turn → locked wall reset last | 6/6 |
| Nonzero start pose → turn → drive → point | 3/6 |
| Timed-out boomerang → continue point → drive | 0/6, injected failure retained |
| Arc → reverse arc | 0/6 |

The user's boomerang/turn and boomerang/point combinations pass all endpoint
checks on the six-motor two-tracker preset only. Their first boomerang is identical
to the previously recorded tracker run. The other presets/sensor layouts retain
deadline, heading or position failures; executing them in order does not repair
those failures.

## State continuity and physical settlement

The runner constructs one bridge per sequence, not one per step. Calls are
synchronous; this adapter does not spawn a background motion or heading-control
task. Each C++ routine constructs its own PID objects, while pose, sensor history,
encoders and physical dynamics continue across steps.

The observer recorded **210 handoffs**. Post-processing independently verified
zero wrapped odometry-heading discontinuity and zero change in stored heading
target during each hold. The original observer also checks position continuity,
ordered timestamps, exact 300 ms holds, absence of powered voltage commands
during holds, and skip behavior. It makes 276,596 frame/event assertions; these
are repeated trace checks, not that many independent physical experiments.

“Stopped” here means commanded zero voltage/Hold, **not exactly zero body speed**:

| Preceding action | Handoffs | Maximum next-entry speed | Maximum absolute yaw rate |
| --- | ---: | ---: | ---: |
| Passed | 150 | 0.013618 in/s | 0.045651°/s |
| Failed, continuation enabled | 60 | 1.209124 in/s | 10.932809°/s |

The failed-step maxima occur in different cases. They demonstrate why continuing
after an unsuccessful motion is a deliberate recovery choice. For the user's
six-motor/tracker boomerang combinations, next-entry residuals are only
0.004698 in/s and 0.005800°/s after the 300 ms hold.

Action meanings also affect the apparent path:

- Point coordinates and turn headings are absolute field values; drive distance
  is relative to the current position and stored heading target.
- A point move does not promise a final body heading. A following drive uses the
  measured heading left by that point move. Add an explicit turn when the next
  drive must face a particular direction.
- A boomerang includes its own final-heading alignment; that rotation is part of
  the boomerang, even if a separate turn follows it.
- After a timeout, safety resets the held heading to the measured heading. With
  continuation enabled, the following drive does not pursue the abandoned turn
  target. Its distance still uses encoder travel; pose accuracy alone cannot
  eliminate slip-related distance errors.

A separate source review found a possible stale previous-output baseline through
a successful stopped `turnToAngle`/`turnToPoint` after a live `exit=false` move.
This is outside these builder sequences, which request stopping on every action.
It was not dynamically reproduced or fixed during this validation pass.

## Replays and reproducibility

In the live builder at `http://127.0.0.1:8765`, scroll to **Inspect the result →
Recorded run** and select one of these fresh records. Play runs the whole timeline
in order; result-row inspection still has the defect described above.

- **Sequence check / Drive-turn-drive**: all three actions pass, 5.500 s including
  holds. Start times are 0, 2.190 and 3.310 seconds.
- **Sequence check / Point-turn-drive**: point `(12,12)`, explicit turn 45°, drive
  12 inches; all pass. Starts 0, 2.680, 3.930 seconds; total 6.120 seconds. The
  final drive error is 1.418 inches / 0.686°.
- **Sequence check / Boomerang-point**: both pass, total 6.250 seconds.
- **Sequence check / Timeout recovery**: deliberately fails its 50 ms turn;
  following drive and turn pass. Total 3.110 seconds. The overall result stays red.

Generated evidence is in ignored `bin/sequence-validation-4uJnhL/`:
`sequence_audit.py`, `results.json`, `summary.json`, per-case normalized specs,
invocation events, complete C++/physics traces, logs and `audit.json` files.
`postcheck.py` and `postcheck.json` preserve the independent heading/settlement
check. `browser/` contains the GET-only Chromium reproduction, expected/actual
JSON and `first-result-row-mismatch.png`. Builder records remain under
`bin/boomerang-continuation-2jKnx4/builder/`.

Commands run:

```sh
python3 -B bin/sequence-validation-4uJnhL/sequence_audit.py
python3 -B bin/sequence-validation-4uJnhL/postcheck.py
python3 -B tests/vexsim/builder/test_builder.py
node tests/vexsim/builder/reference_geometry_test.mjs
node bin/sequence-validation-4uJnhL/browser/replay_inspection_audit.mjs
git diff --check
```

The sequence audit and postcheck exit 0; the separate browser-inspection audit
exits 1 for its three confirmed failures. The existing builder tests pass 17/17,
and the reference-geometry tests pass 206 checks. The mixed-sequence script exits
on scheduling failure; its exit 0 must not be read as all motion endpoints passing.

All physical runs match source fingerprint
`f095fb5197cf9a6377170de219a5a7f96101d1200de2f608ed288798e13ab739`, identical to
the prior passing physics-first `baseline-32f59b6a270e` (238 simulator tests,
one headless GUI skip, plus independent audits). That baseline was verified as
unchanged, not claimed to have been rerun here. The graph navigation was read-only;
direct source inspection and recorded execution established the sequencing facts.
No controller, simulator, frontend, gains, dependencies or lockfiles were changed.
Existing staged edits were preserved. This is simulation validation, not a
firmware-concurrency test or measured hardware certification.
