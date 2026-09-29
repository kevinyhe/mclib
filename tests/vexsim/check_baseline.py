"""Fail if a scenario that passed in the baseline fails now.

The physics matrix has known failures (arcs and some boomerangs, see
BUILDER_VALIDATION.md), so `run.py` exits nonzero on main. CI instead compares
each run against `ci_baseline.json`, the scenarios that passed when it was
last updated. A scenario that starts passing is reported but does not fail
the job; add it to the baseline with `--update`.

    python3 tests/vexsim/check_baseline.py drive /path/to/results.json
    python3 tests/vexsim/check_baseline.py drive /path/to/results.json --update
"""

import argparse
import json
import sys
from pathlib import Path

BASELINE = Path(__file__).with_name("ci_baseline.json")


def main():
    parser = argparse.ArgumentParser(description=__doc__,
                                     formatter_class=argparse.RawDescriptionHelpFormatter)
    parser.add_argument("mode", choices=("drive", "two"), help="--tracking-mode of the run")
    parser.add_argument("results", type=Path, help="results.json written by run.py")
    parser.add_argument("--update", action="store_true",
                        help="write this run's passing scenarios into the baseline")
    args = parser.parse_args()

    rows = json.loads(args.results.read_text())
    passed = {row["name"] for row in rows if row["passed"]}
    ran = {row["name"] for row in rows}
    baseline = json.loads(BASELINE.read_text())

    if args.update:
        baseline[args.mode] = sorted(passed)
        BASELINE.write_text(json.dumps(baseline, indent=2) + "\n")
        print(f"{args.mode}: baseline now {len(passed)} scenarios")
        return 0

    expected = set(baseline[args.mode])
    missing = sorted(expected - ran)
    regressed = sorted((expected & ran) - passed)
    improved = sorted(passed - expected)

    print(f"{args.mode}: {len(passed)}/{len(rows)} passed, baseline {len(expected)}")
    for name in improved:
        print(f"  now passing, not in baseline: {name}")
    for name in missing:
        print(f"  in baseline but did not run: {name}")
    for name in regressed:
        print(f"  REGRESSED: {name}")
    return 1 if regressed or missing else 0


if __name__ == "__main__":
    raise SystemExit(main())
