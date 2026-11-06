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
    or None for the open floor. @p preload is the colour of the half of
    the preload Pin that points up in the closed claw, or "none". Raises
    vexsim's StartOverlapError if the robot starts inside anything."""
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
    from vexsim.override import toggle_outline
    from vexsim.units import INCH
    half = of.HALF
    goals = [dict(name=g.name, kind=g.kind, height_in=round(g.height / INCH, 2),
                  outline=[to_planner(g.x + px, g.y + py) for px, py in of.goal_octagon()],
                  center=to_planner(g.x, g.y)) for g in of.GOALS]
    loaders = []
    for loader in of.LOADERS:
        wall = -loader.inward_x * half
        front = wall + loader.inward_x * of.LOADER_DEPTH
        loaders.append(dict(name=loader.name, alliance=loader.alliance, center=to_planner(loader.x, loader.y),
                            outline=_rect(min(wall, front), max(wall, front),
                                          loader.y - of.LOADER_WIDTH / 2, loader.y + of.LOADER_WIDTH / 2),
                            zone=_rect(*of.load_zone(loader))))
    toggles = [dict(quadrant=t.quadrant, faces=list(t.faces),
                    outline=[to_planner(x, y) for x, y in toggle_outline(t)]) for t in of.TOGGLES]
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
    """The cascade robot's pieces with the lift down and the claw open, and
    where the preload hangs in the closed claw with each half up, inches in
    the robot frame: forward and left of the middle of the drivetrain, and
    up from the floor. A preload's axis is
    (forward, left, up), from its red half to its yellow half. Each piece
    rises `rise_share` of what the claw rises, and a jaw (`jaw_side` 1 on the
    left, -1 on the right) also slides toward the middle as the claw closes.
    A roller piece turns on each of its `hinges`, the axle's first: lines
    across the robot through (forward, up) in its side view. A piece seen
    from the side has a `profile` (forward, up). `frame` is the drivetrain's
    outline and the inside edge of the bumpers around it."""
    from vexsim import override_field as of
    from vexsim.cascade import GEOMETRY as g, CascadeRobot
    from vexsim.presets import override_cascade
    from vexsim.units import INCH
    robot = CascadeRobot()
    chassis = override_cascade()
    inches = lambda value: round(value / INCH, 4)  # noqa: E731
    pairs = lambda points: [[inches(a), inches(b)] for a, b in points]  # noqa: E731
    parts = []
    for component in robot.components(chassis):
        part = component.part
        parts.append(dict(key=component.key, name=part.name, parents=list(component.parents),
                          rise_share=round(component.rise_share, 6), stage=component.stage(g.stages),
                          jaw_side=component.jaw_side,
                          hinges=[dict(name=hinge.name, forward_in=inches(hinge.forward), up_in=inches(hinge.up))
                                  for hinge in component.hinges],
                          outline=pairs(part.corners()), profile=pairs(part.profile),
                          back=inches(part.back), front=inches(part.front), right=inches(part.right),
                          left=inches(part.left), bottom=inches(part.bottom), top=inches(part.top)))
    preloads = {}
    for upper in of.PRELOAD_COLORS["red"]:
        (forward, left, up), axis, _ = robot.preload_pose(chassis, upper, of.PRELOAD_COLORS["red"])
        preloads[upper] = dict(forward_in=inches(forward), left_in=inches(left), up_in=inches(up),
                               axis=[round(value, 4) for value in axis])
    frame = dict(outline=pairs(robot.frame_outline(chassis)), bumper_inside=pairs(robot.bumper_inside(chassis)),
                 bottom=g.frame_bottom, top=g.frame_top)
    return dict(parts=parts, frame=frame, preloads=preloads,
                claw=dict(open_gap_in=g.jaw_open_gap, closed_gap_in=g.jaw_closed_gap, lowest_in=round(g.grip_lowest, 4),
                          highest_in=round(g.grip_highest, 4), middle_ahead_in=inches(robot.claw_forward(chassis))))


def _planner_direction(x, y):
    return [round(-y, 6), round(x, 6)]


def override_solids():
    """What the 3D view needs besides the drawing, inches: the floor and
    walls, which of the field CAD's parts to draw and how, and the Toggles'
    and Loaders' positions in vexsim. A Toggle's `slot` is the way its hubs
    rise in their slots: how far into the field and how far up per inch."""
    from vexsim import override_cad as cad
    from vexsim import override_field as of
    from vexsim.units import INCH
    inches = lambda value: round(value / INCH, 4)  # noqa: E731
    halves = {kind: [dict(rotation=rotation.tolist(), offset_in=offset.tolist()) for rotation, offset in
                     cad.half_frames(kind)] for kind in ("pin", "cup")}
    slot_inward, slot_up = cad.toggle_shape().slot_direction
    return dict(
        wall_height_in=inches(of.WALL_HEIGHT), wall_thickness_in=inches(of.WALL_THICKNESS),
        tile_seams_in=[round(value - of.ORIGIN_IN, 3) for value in of.GRID_IN],
        cad=dict(pin_half="276-9250-001", cup_half="276-9250-002", halves=halves,
                 goal_parts=["276-9250-003", "276-9250-004", "276-9250-005"],
                 loader_parts=["276-9250-031", "276-9250-032", "276-9250-034"], loader_tube="276-9250-031",
                 toggle_parts=["276-9250-021", "276-9250-022", "276-9250-023", "276-9250-024", "276-9250-027"],
                 toggle_bar="276-9250-023", toggle_turning_parts=["276-9250-023", "276-9250-024"]),
        toggles=[dict(quadrant=t.quadrant, axle=to_planner(t.x, t.y), inward=_planner_direction(t.inward_x, t.inward_y),
                      faces=list(t.faces), axle_height_in=inches(of.TOGGLE_AXLE_HEIGHT),
                      face_angles_deg=[round(math.degrees(a), 3) for a in of.TOGGLE_FACE_ANGLES],
                      slot=[round(slot_inward, 6), round(slot_up, 6)]) for t in of.TOGGLES],
        loader=dict(top_in=inches(of.LOADER_TOP_LOWERED),
                    raised_by_in=inches(of.LOADER_TOP_RAISED - of.LOADER_TOP_LOWERED)))


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
