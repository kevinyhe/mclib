"""Editable Metro point targets. Defaults reproduce the original autonomous.

Edit POINTS below, then regenerate the recording with metro_run.py.
Coordinates are local odometry inches, NOT absolute field/true XY. The routine
starts at (0, 0) and resets XY again before its middle-goal approach.
All directions, speeds, deadlines, turns and mechanism commands stay original.
"""
import hashlib
import math

#                     X      Y       Original movement
POINTS = [
    (-9,     24),                  # First collection point
    (-29,   -10),                  # Approach after the first curve
    (-31.5, -27.5),                # Matchloader approach
    (-32.5,  22),                  # Reverse toward long goal
    (38,     23),                  # Middle goal, relative to the later XY reset
]


def correct_waypoints(source, points=None):
    """Change point coordinates only, guarding the reviewed original source."""
    if source.count('void leftSideSevenMiddle()') != 1:
        raise ValueError('Expected one original Metro routine')
    start = source.index('void leftSideSevenMiddle()')
    end = source.index('void rightSideSevenMiddle()', start)
    routine = source[start:end]
    if hashlib.sha256(routine.encode()).hexdigest() != 'c9292d7b842eb0d285e7365978f5ae5d316f309f19297f1ce52cb64ce9da256c':
        raise ValueError('Expected reviewed original Metro autonomous routine')
    targets = POINTS if points is None else points
    if len(targets) != 5 or any(len(p) != 2 or not all(math.isfinite(v) for v in p) for p in targets):
        raise ValueError('Expected five finite (X, Y) point pairs')
    import re
    index = 0

    def replace(match):
        nonlocal index
        x, y = targets[index]
        index += 1
        return f'moveToPoint({x:g}, {y:g},'

    routine = re.sub(r'moveToPoint\([-\d.]+, [-\d.]+,', replace, routine)
    if index != 5:
        raise ValueError('Expected five original moveToPoint calls')
    return source[:start] + routine + source[end:]
