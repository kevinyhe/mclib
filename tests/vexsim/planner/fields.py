"""The fields a routine can run on: what the simulator puts around the robot,
and what the page draws.

Planner coordinates are inches with +Y away from the red Alliance Station.
vexsim's are metres with +x toward blue, so a vexsim point (x, y) is planner
(-y, x).
"""
from pathlib import Path
import sys

GAME_FIELDS = {
    "override": "Override (2026-27)",
    "push_back": "Push Back (2025-26)",
    "open": "Open floor, no walls",
}
NEW_ROUTINE_FIELD = "override"
OLD_FILE_FIELD = "open"


def use_vexsim(vexsim):
    path = str(Path(vexsim).resolve())
    if path not in sys.path:
        sys.path.insert(0, path)


def to_planner(x, y):
    from vexsim.units import INCH
    return [round(-y / INCH, 3), round(x / INCH, 3)]


def make_world(field, sim):
    """The object whose step() advances the drivetrain and everything around it,
    or None for the open floor."""
    if field == "open":
        return None
    if field == "push_back":
        from vexsim import arena
        return arena.push_back(sim)
    if field == "override":
        from vexsim.override import OverrideWorld
        return OverrideWorld(sim)
    raise ValueError(f"unknown field {field!r}")


def _rect(x0, x1, y0, y1):
    return [to_planner(x, y) for x, y in ((x0, y0), (x1, y0), (x1, y1), (x0, y1))]


def override_drawing():
    from vexsim import override_field as of
    from vexsim.override import toggle_obstacle
    from vexsim.units import INCH
    half = of.HALF
    goals = [dict(name=g.name, kind=g.kind, height_in=round(g.height / INCH, 2),
                  outline=[to_planner(g.x + px, g.y + py) for px, py in of.goal_octagon()],
                  center=to_planner(g.x, g.y)) for g in of.GOALS]
    loaders = []
    for loader in of.LOADERS:
        wall = -loader.inward_x * half
        front = wall + loader.inward_x * of.LOADER_DEPTH
        loaders.append(dict(alliance=loader.alliance,
                            outline=_rect(min(wall, front), max(wall, front),
                                          loader.y - of.LOADER_WIDTH / 2, loader.y + of.LOADER_WIDTH / 2),
                            zone=_rect(*of.load_zone(loader))))
    toggles = [dict(quadrant=t.quadrant, faces=list(t.faces),
                    outline=[to_planner(x, y) for x, y in toggle_obstacle(t).points]) for t in of.TOGGLES]
    quadrants = {"left": [(-half, -half), (0, 0), (-half, half)], "right": [(half, -half), (0, 0), (half, half)],
                 "bottom": [(-half, -half), (0, 0), (half, -half)], "top": [(-half, half), (0, 0), (half, half)]}
    mid = of.MIDFIELD_HALF_DIAGONAL
    return dict(
        name=GAME_FIELDS["override"], half_in=round(half / INCH, 3),
        tape=dict(auton_line=[to_planner(-half, half), to_planner(half, -half)],
                  auton_line_width_in=round(of.AUTON_LINE_WIDTH / INCH, 2),
                  diagonal=[to_planner(-half, -half), to_planner(half, half)],
                  midfield=[to_planner(x, y) for x, y in ((mid, 0), (0, mid), (-mid, 0), (0, -mid))]),
        quadrants=[dict(name=name, color=of.QUADRANT_COLOR[name], outline=[to_planner(x, y) for x, y in points])
                   for name, points in quadrants.items()],
        goals=goals, loaders=loaders, toggles=toggles)


def push_back_drawing():
    from vexsim import field as pb
    from vexsim.units import INCH
    solids = [dict(center=to_planner(x, y), radius_in=round(r / INCH, 3)) for x, y, r, _, _ in
              pb.Field().solids(troughs=True)]
    parks = [dict(alliance=z.alliance, outline=_rect(z.x_min, z.x_max, z.y_min, z.y_max)) for z in pb.park_zones()]
    return dict(name=GAME_FIELDS["push_back"], half_in=round(pb.HALF_Y / INCH, 3), solids=solids, park_zones=parks)


def drawings(vexsim):
    use_vexsim(vexsim)
    return dict(override=override_drawing(), push_back=push_back_drawing(), open=None)


def describe_contact(contact, started):
    """One line for a step's results, planner coordinates."""
    x, y = to_planner(contact.peak_x, contact.peak_y)
    return (f"Hit {contact.element} at ({x:.0f}, {y:.0f}) {contact.start - started:.2f} s into the step, "
            f"peak {contact.peak_force:.0f} N, drive current up to {contact.peak_current:.1f} A.")


def contact_dict(contact, started):
    x, y = to_planner(contact.peak_x, contact.peak_y)
    return dict(element=contact.element, part=contact.part, x=x, y=y,
                at_s=round(contact.start - started, 3), duration_s=round(contact.end - contact.start, 3),
                peak_force_n=round(contact.peak_force, 1), peak_current_a=round(contact.peak_current, 2))


def contacts_between(world, start, end):
    if world is None:
        return []
    return [c for c in world.reported_contacts() if start <= c.start <= end]
