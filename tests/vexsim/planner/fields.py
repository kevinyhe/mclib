"""The fields a routine can run on: what the simulator puts around the robot,
and what the page draws.

Planner coordinates are inches with +Y away from the red Alliance Station.
vexsim's are metres with +x toward blue, so a vexsim point (x, y) is planner
(-y, x).
"""
import math
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


def piece_pose(values):
    """[x, y, z, ax, ay, az] in vexsim metres -> planner inches, axis from the
    first colour's half to the second's."""
    from vexsim.units import INCH
    x, y = to_planner(values[0], values[1])
    return [x, y, round(values[2] / INCH, 3), round(-values[4], 3), round(values[3], 3), round(values[5], 3)]


def piece_list(pieces):
    """Every Pin and Cup with its colours and where it is now."""
    return [dict(id=piece.id, kind=piece.kind, colors=list(piece.colors),
                 pose=piece_pose(list(piece.center()) + list(piece.axis())))
            for piece in pieces.pieces()]


def make_world(field, sim, preload="none"):
    """The object whose step() advances the drivetrain and everything around it,
    or None for the open floor. @p preload is the colour of the half of the
    preload Pin that points up, or "none"."""
    if field == "open":
        return None
    if field == "push_back":
        from vexsim import arena
        return arena.push_back(sim)
    if field == "override":
        from vexsim.cascade import CascadeRobot
        from vexsim.override import OverrideWorld
        return OverrideWorld(sim, preload=None if preload == "none" else preload, robot=CascadeRobot())
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
    from vexsim.pieces import Pieces
    layout = Pieces()
    layout.place_layout(of.starting_layout())
    inches = lambda value: round(value / INCH, 3)  # noqa: E731
    return dict(
        name=GAME_FIELDS["override"], half_in=round(half / INCH, 3),
        tape=dict(auton_line=[to_planner(-half, half), to_planner(half, -half)],
                  auton_line_width_in=round(of.AUTON_LINE_WIDTH / INCH, 2),
                  diagonal=[to_planner(-half, -half), to_planner(half, half)],
                  midfield=[to_planner(x, y) for x, y in ((mid, 0), (0, mid), (-mid, 0), (0, -mid))]),
        quadrants=[dict(name=name, color=of.QUADRANT_COLOR[name], outline=[to_planner(x, y) for x, y in points])
                   for name, points in quadrants.items()],
        goals=goals, loaders=loaders, toggles=toggles, pieces=piece_list(layout),
        piece_size=dict(pin_length_in=inches(of.PIN_LENGTH), pin_flange_in=inches(of.PIN_FLANGE_ACROSS_CORNERS),
                        pin_cone_in=inches(of.PIN_CONE_DIAMETER), pin_tip_in=inches(of.PIN_TIP_DIAMETER),
                        cup_length_in=inches(of.CUP_LENGTH), cup_rim_in=inches(of.CUP_RIM_DIAMETER)),
        robot=cascade_drawing(), solid=override_solids())


def cascade_drawing():
    """The cascade robot's shapes, inches. Forward and left are from the
    middle of the drivetrain, except the aligner legs, which are from the
    claw's grip point, and the Toggle arm pivot, which is from the back."""
    from vexsim.cascade import GEOMETRY as g, TOGGLE_ARM_ANGLES
    from vexsim.units import INCH
    legs = [[[round(x / INCH, 4), round(y / INCH, 4)] for x, y in leg] for leg in g.aligner_legs(0.0)]
    return dict(
        frame=dict(bottom_in=g.frame_bottom, top_in=g.frame_top, notch_depth_in=g.front_notch_depth,
                   notch_half_width_in=g.front_notch_half_width),
        rails=dict(bottom_in=g.rail_bottom, length_in=g.rail_length, inner_in=g.rail_inner, outer_in=g.rail_outer,
                   back_in=g.rail_back, front_in=g.rail_front, stages=g.stages),
        claw=dict(ahead_in=g.claw_ahead, jaw_length_in=g.jaw_length, jaw_thickness_in=g.jaw_thickness,
                  below_in=g.jaw_below, above_in=g.jaw_above, slot_in=g.flange_slot, open_gap_in=g.jaw_open_gap,
                  closed_gap_in=g.jaw_closed_gap,
                  lowest_in=g.grip_lowest, highest_in=g.grip_highest),
        carriage=dict(front_in=g.carriage_front, below_in=g.carriage_below, above_in=g.carriage_above),
        aligner=dict(legs_in=legs, top_below_grip_in=g.aligner_top_below_grip,
                     bottom_below_grip_in=g.aligner_bottom_below_grip),
        toggle_arm=dict(pivot_from_back_in=g.toggle_pivot_ahead, pivot_height_in=g.toggle_pivot_height,
                        length_in=g.toggle_arm_length, pad_size_in=g.toggle_pad_size,
                        pad_width_in=g.toggle_pad_width, tower_half_width_in=g.toggle_tower_half_width,
                        stowed_deg=TOGGLE_ARM_ANGLES["stowed"]))


def _planner_direction(x, y):
    return [round(-y, 6), round(x, 6)]


def override_solids():
    """Heights and profiles for the 3D view, inches."""
    from vexsim import override_field as of
    from vexsim.units import INCH
    inches = lambda value: round(value / INCH, 4)  # noqa: E731
    base_radius = (of.GOAL_BASE_ACROSS_FLATS + 2.0 * of.GOAL_BASE_CORNER_RADIUS) / 4.0
    return dict(
        wall_height_in=inches(of.WALL_HEIGHT), wall_thickness_in=inches(of.WALL_THICKNESS),
        tile_seams_in=[round(value - of.ORIGIN_IN, 3) for value in of.GRID_IN],
        goals=[dict(name=g.name, top_of_base_in=inches(g.riser + of.GOAL_BASE_PLATE), height_in=inches(g.height),
                    base_radius_in=inches(base_radius), top_radius_in=inches(of.GOAL_TOP_ACROSS / 2.0),
                    hole_radius_in=inches(of.GOAL_HOLE_DIAMETER / 2.0)) for g in of.GOALS],
        toggles=[dict(quadrant=t.quadrant, axle=to_planner(t.x, t.y), inward=_planner_direction(t.inward_x, t.inward_y),
                      faces=list(t.faces), length_in=inches(of.TOGGLE_LENGTH), axle_height_in=inches(of.TOGGLE_AXLE_HEIGHT),
                      circumradius_in=inches(of.TOGGLE_CIRCUMRADIUS),
                      face_angles_deg=[round(math.degrees(a), 3) for a in of.TOGGLE_FACE_ANGLES]) for t in of.TOGGLES],
        loader=dict(opening_in=inches(of.LOADER_OPENING), top_in=inches(of.LOADER_TOP_LOWERED),
                    bore_in=inches(of.LOADER_BORE)),
        pin_half_profile=[[inches(of.PIN_FLANGE_THICKNESS / 2.0), inches(of.PIN_CONE_DIAMETER / 2.0)],
                          [inches(of.PIN_LENGTH / 2.0 - of.PIN_TIP_STRAIGHT), inches(of.PIN_TIP_DIAMETER / 2.0)],
                          [inches(of.PIN_LENGTH / 2.0), inches(of.PIN_TIP_DIAMETER / 2.0)]],
        pin_flange=dict(thickness_in=inches(of.PIN_FLANGE_THICKNESS), radius_in=inches(of.PIN_FLANGE_ACROSS_FLATS / 2.0)),
        cup_half_profile=[[0.0, inches(of.CUP_WAIST_DIAMETER / 2.0)], [inches(of.CUP_LENGTH / 2.0),
                                                                       inches(of.CUP_RIM_DIAMETER / 2.0)]])


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
