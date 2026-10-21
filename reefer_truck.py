"""Miniature refrigerated rigid truck (cab-over, 4 axles), for 3D printing.

All geometry below is authored at a 100 mm design length; every export is scaled by
SCALE (1.25 -> the 125 mm truck). Values that must hold in real millimetres whatever the scale
(fit clearance, colour inlay depth) are divided by SCALE where they are defined.

Run headless:  freecadcmd.exe reefer_truck.py
Or in the GUI: Macro > Macros... > Execute.
Units are mm. X = length (rear at 0, front at +X), Y = width, Z = up (ground at 0).
"""
import os
import json
import math
import FreeCAD as App
import Part
import Mesh

V = App.Vector

# ---- Parameters ------------------------------------------------------------
TRUCK_LENGTH = float(os.environ.get("TRUCK_LENGTH", "125"))   # mm; another size: TRUCK_LENGTH=130 freecadcmd reefer_truck.py
SCALE = TRUCK_LENGTH / 100.0  # export scale from the 100 mm design
LENGTH = 100.0                # overall design length, including bumper
NOSE = LENGTH - 0.8           # cab front face (bumper sticks out 0.8)
WIDTH = 24.0                  # cab and box width
HW = WIDTH / 2

WHEEL_R = 4.8
FRONT_WHEEL_W = 3.4
REAR_WHEEL_W = 4.0            # dual-tyre look
WHEEL_Y = 7.8                 # inner face of wheels from centreline
AXLE_Z = WHEEL_R
AXLES = [(86.7, FRONT_WHEEL_W), (66.0, FRONT_WHEEL_W), (35.0, REAR_WHEEL_W), (22.4, REAR_WHEEL_W)]

FRAME_W = 13.0
FRAME_Z = (5.5, 8.0)

CAB_X0 = 76.5
CAB_Z0 = 6.5
CAB_ROOF = 28.4               # main cab roof (the deflector on top rises to ROOF_TOP)
ROOF_TOP = 33.2               # top of the roof deflector (it rises to meet the reefer unit)
CAB_WALL = 1.2                # hollow cab wall thickness (3 perimeters at 0.4 mm)
CAB_SPLIT = 11.0              # everything on the cab below this (door-bottom line) is black
CAB_FLOOR = 11.6              # inside floor height: gives the white cab its own 0.6 floor

BOX_X = (1.0, 73.0)           # front wall leaves a 3.5 mm gap to the cab for the reefer unit
BOX_Z = (11.0, 35.2)

DETAIL = 0.4                  # depth of engraved panel lines / recesses

SCRIPT_DIR = os.path.dirname(os.path.abspath(__file__)) if "__file__" in globals() else os.getcwd()
# each length gets its own folder (e.g. 125mm\)
OUT_DIR = os.path.join(SCRIPT_DIR, f"{TRUCK_LENGTH:g}mm")
os.makedirs(OUT_DIR, exist_ok=True)
# Final truck artwork from Aldom/Quisk (Aldom-3D-Truck_Artwork/SVG: 1_Truck-Side_L, 2_Truck-Side_R,
# 3_Truck-Back, 4_Truck-Top-Revised), converted to aldom_final_artwork.json by quisk_svg.py
FINAL = json.load(open(os.path.join(SCRIPT_DIR, "aldom_final_artwork.json")))
# rear doors: the back panel scaled to fit the door height and width; the lock bars break round its logo
K_BACK = min(WIDTH / FINAL["back"]["size"][0], (BOX_Z[1] - BOX_Z[0]) / FINAL["back"]["size"][1])
_rb = [p[1] for f in FINAL["back"]["red"] for sp in f["subpaths"] for seg in sp for p in seg[1:]]
REAR_V = BOX_Z[0] + min(_rb) * K_BACK
REAR_H = (max(_rb) - min(_rb)) * K_BACK


# ---- Helpers ---------------------------------------------------------------
def box(x0, x1, y0, y1, z0, z1):
    return Part.makeBox(x1 - x0, y1 - y0, z1 - z0, V(x0, y0, z0))


def cbox(x0, x1, w, z0, z1):
    return box(x0, x1, -w / 2, w / 2, z0, z1)


def cyl_y(x, z, r, y0, y1):
    return Part.makeCylinder(r, y1 - y0, V(x, y0, z), V(0, 1, 0))


def cyl_x(x0, x1, y, z, r):
    return Part.makeCylinder(r, x1 - x0, V(x0, y, z), V(1, 0, 0))


def prism_xz(pts, y0, y1):
    wire = Part.makePolygon([V(x, y0, z) for x, z in pts] + [V(pts[0][0], y0, pts[0][1])])
    return Part.Face(wire).extrude(V(0, y1 - y0, 0))


def fillet(shape, radii, edge_filter, label=""):
    edges = [e for e in shape.Edges if edge_filter(e)]
    for r in radii:
        try:
            s = shape.makeFillet(r, edges)
            if s.isValid():
                return s
        except Exception:
            pass
    print("  fillet skipped:", label)
    return shape


def fuse_all(shapes):
    s = shapes[0]
    for o in shapes[1:]:
        s = s.fuse(o)
    return s.removeSplitter()


def mirror_y(shape):
    return shape.mirror(V(0, 0, 0), V(0, 1, 0))


# ---- Cab -------------------------------------------------------------------
# Side profile as in the photo: vertical lower front, raked windscreen curving into the
# roof with a large radius, rounded front corners, and a raised roof deflector at the rear.
WS_BOTTOM = 15.0              # windscreen starts here (about halfway up the cab front, as in the photo)
WS_RAKE = 0.30                # windscreen leans back 0.3 per unit of height (measured on the photo)
ROOF_R = 4.0                  # windscreen-to-roof radius
CORNER_R = 2.0                # front and roof side edges
def ws_x(z):  # x of windscreen surface at height z
    return NOSE - (z - WS_BOTTOM) * WS_RAKE

cab_profile = [(CAB_X0, CAB_Z0), (NOSE, CAB_Z0), (NOSE, WS_BOTTOM), (ws_x(CAB_ROOF), CAB_ROOF), (CAB_X0, CAB_ROOF)]


def cab_body(profile_face, hw, roof_r, front_r, corner_r, label):
    """Extrude a side profile (face in the XZ plane) to +-hw and round it like the cab:
    windscreen-to-roof, front-to-windscreen and the side edges (not the base or back)."""
    body = profile_face.extrude(V(0, 2 * hw, 0))
    body.translate(V(0, -hw, 0))
    bb = body.BoundBox
    across = lambda e: e.BoundBox.YLength > 2 * hw - 0.1
    body = fillet(body, (roof_r, roof_r * 0.8, roof_r * 0.6), lambda e: across(e) and e.BoundBox.ZMin > bb.ZMax - 0.1
                  and e.BoundBox.XMin > bb.XMin + 1, label + " windscreen to roof")
    if front_r:
        body = fillet(body, (front_r, front_r * 0.6), lambda e: across(e) and abs(e.BoundBox.ZMin - WS_BOTTOM) < 0.1,
                      label + " front to windscreen")
    return fillet(body, (corner_r, corner_r * 0.75, corner_r * 0.6), lambda e: e.BoundBox.YLength < 1e-6
                  and e.BoundBox.ZMax > bb.ZMin + 0.1 and e.BoundBox.XMax > bb.XMin + 0.1, label + " sides")


profile_face = Part.Face(Part.makePolygon([V(x, 0, z) for x, z in cab_profile] + [V(cab_profile[0][0], 0, cab_profile[0][1])]))
cab = cab_body(profile_face, HW, ROOF_R, 0.8, CORNER_R, "cab")

# Hollow interior: the same body built from the profile shrunk by the wall thickness, so
# the wall stays CAB_WALL thick round the curves too (a 3D offset of the rounded shell
# is not reliable in FreeCAD)
t = CAB_WALL
inner = cab_body(profile_face.makeOffset2D(-t, join=2), HW - t, ROOF_R - t, 0, CORNER_R - t, "cavity")
cavity = inner.common(box(CAB_X0 - 1, LENGTH + 1, -HW - 1, HW + 1, CAB_FLOOR, 50))
cab = cab.cut(cavity)

# Roof deflector: rises in a smooth curve from mid-roof to ROOF_TOP at the back
DEFL_HW = 9.5                 # half width (narrower than the cab, as in the front view)
# The deflector's back reaches past the cab to the reefer unit's front face, leaving only
# a 0.1 mm glue gap, so the top of the cab is braced against the body.
CAB_BRACE_X = (CAB_X0 - 0.3) + 0.1 / SCALE          # reefer front face (GAP_FRONT) + glue gap
DEFL_X = (CAB_BRACE_X, 90.0)  # back and front of the deflector
bz = Part.BezierCurve()
bz.setPoles([V(DEFL_X[1], 0, CAB_ROOF - 0.4), V(88.5, 0, CAB_ROOF - 0.2), V(86.5, 0, ROOF_TOP), V(84.0, 0, ROOF_TOP)])
defl_wire = Part.Wire([Part.LineSegment(V(DEFL_X[0], 0, CAB_ROOF - 0.4), V(DEFL_X[1], 0, CAB_ROOF - 0.4)).toShape(),
                       bz.toShape(),
                       Part.LineSegment(V(84.0, 0, ROOF_TOP), V(DEFL_X[0], 0, ROOF_TOP)).toShape(),
                       Part.LineSegment(V(DEFL_X[0], 0, ROOF_TOP), V(DEFL_X[0], 0, CAB_ROOF - 0.4)).toShape()])
deflector = Part.Face(defl_wire).extrude(V(0, 2 * DEFL_HW, 0))
deflector.translate(V(0, -DEFL_HW, 0))
deflector = fillet(deflector, (1.5, 1.0, 0.6), lambda e: e.BoundBox.YLength < 1e-6 and e.BoundBox.ZMin > CAB_ROOF
                   and e.BoundBox.XMax > DEFL_X[0] + 0.1, "deflector sides")
cab = cab.fuse(deflector)

# Windows are solid (Aldom feedback): black glass panels sitting flush in shallow pockets.
# In the kit they are separate flat black pieces, so the white cab has no colour changes.
GLASS_T = 0.6 / SCALE         # glass 0.6 mm thick, in 0.6 mm deep pockets
GLASS_CLEAR = 0.1 / SCALE     # fit clearance round each panel (kit)
ws_n = V(1, 0, WS_RAKE).normalize()                  # windscreen's outward normal
# windscreen glass covers the flat part of the windscreen: above the small front radius,
# below the roof curve, and clear of the rounded front corners
WS_GLASS_Z = (WS_BOTTOM + 0.4, 25.3)
WS_GLASS_HW = HW - CORNER_R - 0.3


def ws_panel(grow, out, depth):
    """A panel lying in the windscreen plane, grown by `grow` all round, from `out` outside
    the windscreen surface to `depth` inside it."""
    dz = grow * ws_n.x                               # growing along the slope, expressed in z
    z0, z1, w = WS_GLASS_Z[0] - dz, WS_GLASS_Z[1] + dz, WS_GLASS_HW + grow
    pts = [V(ws_x(z0), -w, z0), V(ws_x(z0), w, z0), V(ws_x(z1), w, z1), V(ws_x(z1), -w, z1)]
    face = Part.Face(Part.makePolygon(pts + [pts[0]]))
    face.translate(ws_n * out)
    solid = face.extrude(ws_n * -(out + depth))
    return fillet(solid, (0.6, 0.4), lambda e: abs(e.Length - (out + depth)) < 1e-6, "windscreen glass corners")


glass_ws = ws_panel(0.0, 0.0, GLASS_T)
cab = cab.cut(glass_ws)

# Cab sides, measured on the photo's side view: a door whose front edge leans back with the
# windscreen pillar, a side window with a raked front edge and a bottom edge that drops
# towards the front, a handle at the door's rear edge, and behind the door a tall recessed
# panel over a locker hatch.
DOOR_BOTTOM, DOOR_TOP = 11.3, CAB_ROOF - 2.2        # door sits on the black lower cab; the side is flat up to the roof-edge rounding
DOOR_REAR = 82.3
DOOR_LEAN = 0.19                                    # front edge: x drops this much per unit of height


def door_front(z):
    return 96.4 - (z - DOOR_BOTTOM) * DOOR_LEAN


def xz_face(pts, round_r=0.0):
    """Polygon in the XZ plane (x, z points); corners rounded to round_r if given."""
    f = Part.Face(Part.makePolygon([V(x, 0, z) for x, z in pts] + [V(pts[0][0], 0, pts[0][1])]))
    if round_r:
        f = f.makeOffset2D(-round_r, join=2).makeOffset2D(round_r, join=0)
    return f


def outline(face, width=0.4):
    """A groove of the given width running round the inside of a shape."""
    return face.cut(face.makeOffset2D(-width, join=0))


def into_sides(face, depth):
    """Extrude an XZ-plane shape into both cab sides, `depth` deep from the side faces."""
    solid = face.extrude(V(0, depth + 1, 0))
    solid.translate(V(0, HW - depth, 0))
    return solid.fuse(mirror_y(solid))


door_face = xz_face([(DOOR_REAR, DOOR_BOTTOM), (door_front(DOOR_BOTTOM), DOOR_BOTTOM),
                     (door_front(DOOR_TOP), DOOR_TOP), (DOOR_REAR, DOOR_TOP)], 0.6)
WIN_TOP, WIN_LOW_REAR, WIN_LOW_FRONT = DOOR_TOP - 0.6, 20.1, 17.1
win_face = xz_face([(DOOR_REAR + 0.7, WIN_LOW_REAR), (door_front(WIN_LOW_FRONT) - 0.8, WIN_LOW_FRONT),
                    (door_front(WIN_TOP) - 0.8, WIN_TOP), (DOOR_REAR + 0.7, WIN_TOP)], 0.5)
upper_panel = xz_face([(CAB_X0 + 0.9, 16.8), (DOOR_REAR - 0.8, 16.8), (DOOR_REAR - 0.8, DOOR_TOP - 0.2),
                       (CAB_X0 + 0.9, DOOR_TOP - 0.2)], 0.6)
hatch = xz_face([(CAB_X0 + 0.9, DOOR_BOTTOM + 0.3), (DOOR_REAR - 0.8, DOOR_BOTTOM + 0.3),
                 (DOOR_REAR - 0.8, 16.2), (CAB_X0 + 0.9, 16.2)], 0.5)
handle = xz_face([(DOOR_REAR + 0.5, 12.6), (DOOR_REAR + 1.3, 12.6), (DOOR_REAR + 1.3, 13.4),
                  (DOOR_REAR + 0.5, 13.4)])
grooves = fuse_all([outline(door_face), outline(upper_panel, 0.3), outline(hatch, 0.3), handle])
cab = cab.cut(into_sides(grooves, DETAIL))
# side windows: black glass panels flush in pockets (one shape, mirrored for the right side)
glass_side = win_face.extrude(V(0, GLASS_T, 0))
glass_side.translate(V(0, HW - GLASS_T, 0))
cab = cab.cut(glass_side).cut(mirror_y(glass_side))
glass_all = fuse_all([glass_ws, glass_side, mirror_y(glass_side)])

# Solid mirrors (Aldom feedback): one block joined to the cab side along its whole height,
# instead of a head on two thin arms. Its underside slopes at 45 degrees so it prints
# without support.
MIRROR_X = (95.0, 96.0)
mirror_prof = [(HW - 1.1, 15.8), (HW + 2.7, 15.8 + 3.8), (HW + 2.7, 24.2), (HW - 1.1, 24.2)]
mirror_face = Part.Face(Part.makePolygon([V(MIRROR_X[0], y, z) for y, z in mirror_prof] +
                                         [V(MIRROR_X[0], mirror_prof[0][0], mirror_prof[0][1])]))
mirror_block = mirror_face.extrude(V(MIRROR_X[1] - MIRROR_X[0], 0, 0))
cab = cab.fuse(mirror_block).fuse(mirror_y(mirror_block))

# ---- Cab interior ------------------------------------------------------------
# With solid windows the interior can't be seen, so there are no seats, dashboard or
# steering wheel any more; only the bosses for the locating dowels remain.
parts = []
# Bosses for the dowels that locate the cab on the chassis (kit)
CAB_DOWELS = [(79.0, -7.5), (79.0, 7.5)]
for x, y in CAB_DOWELS:
    parts.append(Part.makeCylinder(1.5, 2.0, V(x, y, CAB_FLOOR - 0.1)))
cab = fuse_all([cab] + parts)


# ---- Refrigerated box ------------------------------------------------------
cargo = box(BOX_X[0], BOX_X[1], -HW, HW, BOX_Z[0], BOX_Z[1])
cargo = fillet(cargo, (0.6, 0.4), lambda e: True)
# Aluminium frame: engraved lines round the side panels. The sides stay flat so the
# box can print lying on its side (see the kit section).
frame_lines = []
for y0, y1 in ((HW - 0.3, HW + 1), (-HW - 1, -HW + 0.3)):
    for z in (BOX_Z[0] + 0.6, BOX_Z[1] - 1.0):
        frame_lines.append(box(BOX_X[0] + 0.6, BOX_X[1] - 0.6, y0, y1, z, z + 0.4))
    frame_lines.append(box(BOX_X[1] - 1.1, BOX_X[1] - 0.7, y0, y1, BOX_Z[0] + 0.6, BOX_Z[1] - 0.6))
cargo = cargo.cut(fuse_all(frame_lines))
# Rear doors: centre split and hinge lines
for y in (-0.2, -HW + 2.2, HW - 2.6):
    cargo = cargo.cut(box(BOX_X[0] - 1, BOX_X[0] + DETAIL, y, y + 0.4, BOX_Z[0] + 1.2, BOX_Z[1] - 1.2))
# Rear door lock bars, broken where the rear lettering is so it stays legible
for y in (-3.5, -1.5, 1.1, 3.1):
    for z0, z1 in ((BOX_Z[0] + 1.5, REAR_V - 0.4), (REAR_V + REAR_H + 0.4, BOX_Z[1] - 1.5)):
        cargo = cargo.fuse(box(BOX_X[0] - 0.35, BOX_X[0] + 0.1, y, y + 0.4, z0, z1))

# Reefer (fridge) unit, as in the photo: an upright housing on the front wall in the gap
# behind the cab. The roof deflector rises to meet its front face, so from the front only
# its top shows above the cab, with a louvred black grille panel; vertical vents on its sides.
REEFER_HW = 5.5
REEFER_TOP = BOX_Z[1] - 0.6                          # below the box's rounded top edge
REEFER_BOTTOM = 28.6
REEFER_SEAT = 0.4                                    # sits this deep in a recess in the box wall
GAP_FRONT = CAB_X0 - 0.3                             # just clears the back of the cab
reefer = box(BOX_X[1] - REEFER_SEAT, GAP_FRONT, -REEFER_HW, REEFER_HW, REEFER_BOTTOM, REEFER_TOP)
reefer = fillet(reefer, (0.8, 0.5, 0.3), lambda e: e.BoundBox.XMin > BOX_X[1] + 0.1 and
                (e.BoundBox.ZMin > REEFER_TOP - 0.1 or e.BoundBox.XMin > GAP_FRONT - 0.01), "reefer")
# black grille on the front face, above the deflector: one panel of horizontal louvres,
# inlaid into the white housing
GRILLE_CELLS = ((-4.4, 4.4),)                        # a single black panel
GRILLE_Z = (ROOF_TOP + 0.25, ROOF_TOP + 1.45)
# 1.2 mm deep: the panel is so small that at 0.6 mm its file is under the ~9 mm3 at which
# slicers decide a model must be in inches and scale it up 25.4x
GRILLE_DEPTH = 1.2 / SCALE
grille_in = fuse_all([box(GAP_FRONT - GRILLE_DEPTH, GAP_FRONT + 1, y0, y1, GRILLE_Z[0], GRILLE_Z[1])
                      for y0, y1 in GRILLE_CELLS])
reefer_black = grille_in.common(reefer).removeSplitter()
reefer = reefer.cut(grille_in).removeSplitter()
for y0, y1 in GRILLE_CELLS:
    for z in (ROOF_TOP + 0.45, ROOF_TOP + 0.95):
        reefer_black = reefer_black.cut(box(GAP_FRONT - 0.2, GAP_FRONT + 1, y0 + 0.2, y1 - 0.2, z, z + 0.3))
# vertical vents on both sides, towards the front
for side in (1, -1):
    for i in range(4):
        x = GAP_FRONT - 2.8 + i * 0.6
        y0, y1 = (REEFER_HW - DETAIL, REEFER_HW + 1) if side > 0 else (-REEFER_HW - 1, -REEFER_HW + DETAIL)
        reefer = reefer.cut(box(x, x + 0.3, y0, y1, REEFER_BOTTOM + 0.6, ROOF_TOP - 0.4))
cargo = cargo.fuse(reefer).removeSplitter()
# in the kit the reefer is its own piece; this block is what it occupies (plus clearance)
reefer_envelope = box(BOX_X[1] - REEFER_SEAT - 0.02, GAP_FRONT + 1, -REEFER_HW - 0.15 / SCALE, REEFER_HW + 0.15 / SCALE,
                      REEFER_BOTTOM - 0.15 / SCALE, REEFER_TOP + 0.15 / SCALE)

# ---- Branding: inlays set flush into the surface (for multi-colour printing) ------
# All artwork comes from Quisk's final panels (FINAL, see above): red, black and grey layers,
# plus the white panel behind the roof QR code.
INLAY_DEPTH = 0.6 / SCALE     # 0.6 mm deep in the printed model
MIN_FEATURE = 0.22 / SCALE    # narrowest line a 0.2 mm nozzle prints reliably (0.22 mm)
def poly_face(pts):
    return Part.Face(Part.makePolygon([V(u, v, 0) for u, v in pts] + [V(pts[0][0], pts[0][1], 0)]))


thin_report = []              # (label, shapes checked, shapes with a part thinner than MIN_FEATURE, unchecked)


def too_thin(face):
    """True if the shape has a stroke thinner than MIN_FEATURE: shrinking it by half of
    MIN_FEATURE then makes part of it vanish or split off. None if the check failed."""
    def poly(w):
        return Part.Face(Part.makePolygon(w.discretize(Deflection=0.002)))
    try:
        # Offsetting curved outlines with holes is fragile: use fine polygons, shrink the
        # outline and grow each hole (the same as shrinking the whole shape).
        eroded = poly(face.OuterWire).makeOffset2D(-MIN_FEATURE / 2, join=2)
        for w in face.Wires:
            if not w.isSame(face.OuterWire):
                eroded = eroded.cut(poly(w).makeOffset2D(MIN_FEATURE / 2, join=2))
        return len(eroded.Faces) != 1
    except Exception:
        return None


def check_printable(label, shapes):
    faces = [f for s in shapes for f in s.Faces]          # text lines hold one face per letter
    results = [too_thin(f) for f in faces]
    thin_report.append((label, len(faces), results.count(True), results.count(None)))


# -- Final artwork on the box: each panel scaled uniformly (never stretched) to fit its face.
#    Panel coordinates: x to the right and v up as seen from outside; the top panel is drawn
#    with the truck's front at the top. The sides and roof stop 2.5 short of the rear end so
#    no colour lands on the rear door panel's edges (they print upright in the kit).
ART_X = (BOX_X[0] + 2.5, BOX_X[1])
K_SIDE = (ART_X[1] - ART_X[0]) / FINAL["side_left"]["size"][0]
SIDE_Z0 = BOX_Z[0] + ((BOX_Z[1] - BOX_Z[0]) - FINAL["side_left"]["size"][1] * K_SIDE) / 2
K_TOP = (ART_X[1] - ART_X[0]) / FINAL["top"]["size"][1]


def art_faces(panel, colour, k):
    """One colour of a final-artwork panel as faces in the XY plane, scaled by k."""
    faces = []
    for fill in FINAL[panel].get(colour, []):
        wires = []
        for sp in fill["subpaths"]:
            edges = []
            for seg in sp:
                pts = [V(q[0] * k, q[1] * k, 0) for q in seg[1:]]
                if seg[0] == "L":
                    if (pts[1] - pts[0]).Length > 1e-7:
                        edges.append(Part.LineSegment(pts[0], pts[1]).toShape())
                else:
                    bz = Part.BezierCurve()
                    bz.setPoles(pts)
                    edges.append(bz.toShape())
            wires.append(Part.Wire(edges))
        faces += Part.makeFace(wires, "Part::FaceMakerBullseye").Faces
    return faces


def place_art(faces, origin, u_dir, v_dir):
    """Map XY artwork onto a face of the truck (X along u_dir, Y along v_dir, from origin) and
    inlay it INLAY_DEPTH deep. u_dir x v_dir is the outward normal, so it reads correctly
    (not mirrored) from outside."""
    normal = u_dir.cross(v_dir)
    m = App.Matrix(u_dir.x, v_dir.x, normal.x, origin.x,
                   u_dir.y, v_dir.y, normal.y, origin.y,
                   u_dir.z, v_dir.z, normal.z, origin.z, 0, 0, 0, 1)
    solids = []
    for f in faces:
        g = f.copy()
        g.transformShape(m)
        g.translate(normal * 0.05)
        solids.append(g.extrude(normal * -(INLAY_DEPTH + 0.05)))
    return fuse_all(solids) if solids else Part.Shape()


UP, FWD = V(0, 0, 1), V(1, 0, 0)
FACES = {   # panel: (origin of its bottom-left corner, u direction, v direction)
    "side_left": (V(ART_X[1], HW, SIDE_Z0), V(-1, 0, 0), UP),     # front at the viewer's left
    "side_right": (V(ART_X[0], -HW, SIDE_Z0), FWD, UP),           # front at the viewer's right
    "back": (V(BOX_X[0], FINAL["back"]["size"][0] * K_BACK / 2, BOX_Z[0]), V(0, -1, 0), UP),
    "top": (V(ART_X[0], FINAL["top"]["size"][0] * K_TOP / 2, BOX_Z[1]), V(0, -1, 0), FWD),  # front at the top
}
K = {"side_left": K_SIDE, "side_right": K_SIDE, "back": K_BACK, "top": K_TOP}
art = {(pn, c): art_faces(pn, c, K[pn]) for pn in FACES for c in ("red", "black", "grey")}

# Roof: the QR code (aldom_qr.json, decodes to https://aldom.com.au/) sits on the artwork's
# white rounded panel, which is kept clear of any red
QR = json.load(open(os.path.join(SCRIPT_DIR, "aldom_qr.json")))
_q, _w = FINAL["top"]["qr"], FINAL["top"]["rounded_white"][0]
assert _q["modules"] == len(QR)
QR_MODULE = _q["pitch"] * K_TOP
white_panel = poly_face([(_w["x"] * K_TOP, _w["v"] * K_TOP), ((_w["x"] + _w["w"]) * K_TOP, _w["v"] * K_TOP),
                        ((_w["x"] + _w["w"]) * K_TOP, (_w["v"] + _w["h"]) * K_TOP),
                        (_w["x"] * K_TOP, (_w["v"] + _w["h"]) * K_TOP)])
white_panel = white_panel.makeOffset2D(-_w["r"] * K_TOP, join=2).makeOffset2D(_w["r"] * K_TOP, join=0)
art[("top", "red")] = [g for f in art[("top", "red")] for g in f.cut(white_panel).Faces]
qr_faces = []                  # runs of black modules; row 0 at the top edge ("v" is the code's top)
for r, row in enumerate(QR):
    c = 0
    while c < len(row):
        if row[c]:
            c1 = c
            while c1 < len(row) and row[c1]:
                c1 += 1
            x0, x1 = _q["x"] * K_TOP + c * QR_MODULE, _q["x"] * K_TOP + c1 * QR_MODULE
            y1 = _q["v"] * K_TOP - r * QR_MODULE
            qr_faces.append(poly_face([(x0, y1 - QR_MODULE), (x1, y1 - QR_MODULE), (x1, y1), (x0, y1)]))
            c = c1
        else:
            c += 1
art[("top", "black")] = qr_faces

for label, key in (("left side red", ("side_left", "red")), ("left side black", ("side_left", "black")),
                   ("left side grey", ("side_left", "grey")), ("right side red", ("side_right", "red")),
                   ("right side black", ("side_right", "black")), ("right side grey", ("side_right", "grey")),
                   ("back red", ("back", "red")), ("roof red", ("top", "red")), ("roof QR", ("top", "black"))):
    if art[key]:
        check_printable(label, art[key])


def inlay(panels, colour):
    return fuse_all([place_art(art[(pn, colour)], *FACES[pn]) for pn in panels if art[(pn, colour)]] or [Part.Shape()])


BOX_PANELS = ("side_left", "side_right", "back")
inlays_red = inlay(BOX_PANELS, "red")
inlays_black = inlay(BOX_PANELS, "black")
inlays_grey = inlay(BOX_PANELS, "grey").cut(inlays_red)     # never two colours in one place
roof_red_in, roof_black_in = inlay(("top",), "red"), inlay(("top",), "black")
red_parts = inlays_red.common(cargo).removeSplitter()      # keeps door grooves as grooves
black_text = inlays_black.common(cargo).removeSplitter()
grey_parts = inlays_grey.common(cargo).removeSplitter()
roof_red = roof_red_in.common(cargo).removeSplitter()
roof_black = roof_black_in.common(cargo).removeSplitter()
cargo = cargo.cut(inlays_red).cut(inlays_black).cut(inlays_grey).cut(roof_red_in).cut(roof_black_in).removeSplitter()

# In the kit the roof artwork is a separate panel, printed face up (all its colour in the top
# layers) and glued into a recess in the box top: on the box, which prints on its side, the
# roof is an upright face and its red diagonal and QR code would need hundreds of colour changes.
ROOF_PANEL_T = 0.8
ROOF_PANEL = (ART_X[0] - 0.2, BOX_X[1] - 1.0, HW - 0.8)   # x from, x to, half width (inside the rounded edges)


def roof_slab(grow, z0, z1):
    x0, x1, hw = ROOF_PANEL
    slab = box(x0 - grow, x1 + grow, -hw - grow, hw + grow, z0, z1)
    return fillet(slab, (0.6 + grow, 0.4 + grow), lambda e: e.BoundBox.ZLength > z1 - z0 - 0.01, "roof panel corners")


roof_panel = roof_slab(0.0, BOX_Z[1] - ROOF_PANEL_T, BOX_Z[1])

# ---- Chassis & running gear (black) -----------------------------------------
frame = cbox(1.0, 99.0, FRAME_W, FRAME_Z[0], FRAME_Z[1])
subframe = cbox(2.0, BOX_X[1] - 1, FRAME_W, FRAME_Z[1] - 0.1, BOX_Z[0] + 0.1)
BUMPER_Z = (2.8, 8.5)
bumper = cbox(97.5, LENGTH, WIDTH + 0.4, BUMPER_Z[0], BUMPER_Z[1])
bumper = fillet(bumper, (0.6, 0.4), lambda e: e.BoundBox.XMin > LENGTH - 0.1)
# recesses in its face, as in the photo: headlights at the upper corners, fog lights
# below them, number plate in the middle (paint the lights silver)
lamps = []
for dy in (-1, 1):
    lamps.append(box(LENGTH - DETAIL, LENGTH + 1, dy * 6.6 - 1.7, dy * 6.6 + 1.7, 5.9, 7.8))   # headlight
    lamps.append(box(LENGTH - DETAIL, LENGTH + 1, dy * 8.2 - 0.6, dy * 8.2 + 0.6, 3.9, 4.6))   # fog light
lamps.append(box(LENGTH - DETAIL, LENGTH + 1, -3.2, 3.2, 3.3, 4.5))                           # number plate
bumper = bumper.cut(fuse_all(lamps))
# rear underrun bar, with a second bar directly in front of it (Aldom feedback): twice as thick
underrun = cbox(0.0, 2.0, WIDTH - 2, 3.5, 5.5).fuse(cbox(2.0, 4.0, WIDTH - 2, 3.5, 5.5))
underrun_brackets = [box(1.0, 3.0, y - 0.6, y + 0.6, 5.0, FRAME_Z[0] + 0.1) for y in (-5.0, 5.0)]
rear_lights = [box(0.0, 0.8, y0, y0 + 2.5, 5.5, 6.4) for y0 in (-10.5, 8.0)]
side_box = cbox(42.5, 59.5, WIDTH - 2, 5.0, BOX_Z[0] + 0.1)                 # skirt / battery box
mudguards = [cbox(16.0, 41.5, WIDTH - 0.4, 10.2, BOX_Z[0] + 0.1),         # tandem
             cbox(59.5, 72.5, WIDTH - 0.4, 10.2, BOX_Z[0] + 0.1)]         # 2nd axle
tanks = []
for side in (1, -1):
    t = cyl_x(71.6, CAB_X0 - 0.3, side * 8.4, 7.4, 2.4)   # ends just behind the cab
    t = fillet(t, (0.6, 0.3), lambda e: True)
    tanks.append(t)
    for bx in (72.3, 75.0):  # straps tying the tank to the frame
        tanks.append(box(bx, bx + 0.8, min(side * 5.5, side * 10.9), max(side * 5.5, side * 10.9), 6.0, 8.0))
chassis = fuse_all([frame, subframe, bumper, underrun] + underrun_brackets + rear_lights +
                   [side_box] + mudguards + tanks)

# Wheel arches through cab and chassis extras
arch = fuse_all([cyl_y(x, AXLE_Z, WHEEL_R + 0.6, -HW - 2, HW + 2) for x, _ in AXLES])
cab = cab.cut(arch).removeSplitter()
chassis = chassis.cut(arch.cut(cbox(0, LENGTH, FRAME_W, 0, 40))).removeSplitter()  # keep the frame rails

# Wheels: black tyres with silver rims (the recessed wheel face, hub cap and nuts)
RIM_R = WHEEL_R - 1.3
RIM_RECESS = 0.3 / SCALE      # wheel face recessed 0.3 mm into the tyre
RIM_SILVER = 0.4 / SCALE      # silver layer under it (fewer mixed-colour layers when printing)
wheels, rims = [], []
wheel_protos = {}             # tyre width -> (x, black tyre, silver rim) on the +Y side, for the kit
for x, w in AXLES:
    for side in (1, -1):
        tyre = cyl_y(x, AXLE_Z, WHEEL_R, WHEEL_Y, WHEEL_Y + w)
        # round only the outer edge: the inner face prints on the plate and needs a flat, full-size base
        tyre = fillet(tyre, (0.6, 0.4), lambda e: e.BoundBox.YLength < 1e-6 and e.BoundBox.YMin > WHEEL_Y + 0.01)
        outer = WHEEL_Y + w
        tyre = tyre.cut(cyl_y(x, AXLE_Z, RIM_R, outer - RIM_RECESS, outer + 1))           # rim recess
        tyre = tyre.fuse(cyl_y(x, AXLE_Z, 1.5, outer - RIM_RECESS - 0.1, outer + 0.2))   # hub cap
        for k in range(6):                                                               # wheel nuts
            import math
            a = k * math.pi / 3
            tyre = tyre.fuse(cyl_y(x + 2.3 * math.cos(a), AXLE_Z + 2.3 * math.sin(a), 0.3,
                                   outer - RIM_RECESS - 0.1, outer))
        tyre = tyre.removeSplitter()
        rim_zone = cyl_y(x, AXLE_Z, RIM_R, outer - RIM_RECESS - RIM_SILVER, outer + 1)   # silver face
        rim = tyre.common(rim_zone).removeSplitter()
        tyre = tyre.cut(rim_zone).removeSplitter()
        wheel_protos.setdefault(w, (x, tyre, rim))
        wheels.append(tyre if side > 0 else mirror_y(tyre))
        rims.append(rim if side > 0 else mirror_y(rim))
    wheels.append(cyl_y(x, AXLE_Z, 1.0, -WHEEL_Y - 0.5, WHEEL_Y + 0.5))  # axle
running_gear = fuse_all([chassis] + wheels)
silver_parts = fuse_all(rims)

# No branding on the cab (Aldom feedback): all of it is on the refrigerated body.

# Lower cab is black, as in the photo: the front below the white panel (grille band), the
# wheel arch trim and the step under the door. The cab is split at the door-bottom line and
# the lower part joins the black chassis piece. Its top is level with the chassis' flat top,
# so the chassis still prints upside down flat on the plate, and the white cab keeps a flat
# base with no colour changes.
cab_lower = cab.common(box(CAB_X0 - 1, LENGTH + 1, -HW - 3, HW + 3, 0, CAB_SPLIT)).removeSplitter()
cab = cab.cut(box(CAB_X0 - 1, LENGTH + 1, -HW - 3, HW + 3, 0, CAB_SPLIT)).removeSplitter()
grille = [cbox(NOSE - DETAIL, NOSE + 1, 15.0, z, z + 0.3) for z in (9.2, 9.8, 10.4)]   # slats above the bumper
cab_lower = cab_lower.cut(fuse_all(grille))
running_gear = running_gear.fuse(cab_lower).removeSplitter()

# ---- Document & export (scaled to SCALE) -------------------------------------
def scaled(shape):
    c = shape.copy()
    c.scale(SCALE)
    return c


all_red = red_parts.fuse(roof_red).removeSplitter()
all_black_text = black_text.fuse(reefer_black).fuse(roof_black).fuse(glass_all).removeSplitter()
white_shape = cab.fuse(cargo).removeSplitter()
# Multi-colour STLs must not overlap, or the slicer has to guess which colour wins:
# trim the black running gear where the frame runs up inside the white cab and box.
black_shape = running_gear.cut(white_shape).fuse(all_black_text).removeSplitter()
whole = fuse_all([running_gear, cab, cargo, all_red, all_black_text, grey_parts, silver_parts])

doc = App.newDocument("ReeferTruck")
objs = {"Cab": cab, "CargoBox": cargo, "Chassis": running_gear, "Branding": all_red,
        "BrandingBlack": all_black_text, "BrandingGrey": grey_parts, "Rims": silver_parts}
for name, shp in objs.items():
    doc.addObject("Part::Feature", name).Shape = scaled(shp)
if App.GuiUp:
    doc.Cab.ViewObject.ShapeColor = (0.95, 0.95, 0.95)
    doc.CargoBox.ViewObject.ShapeColor = (0.95, 0.95, 0.95)
    doc.Chassis.ViewObject.ShapeColor = (0.12, 0.12, 0.12)
    doc.Branding.ViewObject.ShapeColor = (0.85, 0.1, 0.1)
    doc.BrandingBlack.ViewObject.ShapeColor = (0.12, 0.12, 0.12)
    doc.BrandingGrey.ViewObject.ShapeColor = (0.78, 0.78, 0.78)
    doc.Rims.ViewObject.ShapeColor = (0.75, 0.76, 0.78)
doc.recompute()
doc.saveAs(os.path.join(OUT_DIR, "ReeferTruck.FCStd"))

for fname, shp, tol in (("onepiece", whole, 0.02), ("white", white_shape, 0.02),
                        ("black", black_shape, 0.02), ("red", all_red, 0.01),
                        ("grey", grey_parts, 0.01), ("silver", silver_parts, 0.01)):
    tmp = doc.addObject("Part::Feature", "Export_" + fname)
    tmp.Shape = scaled(shp)
    Mesh.export([tmp], os.path.join(OUT_DIR, f"ReeferTruck_{fname}.stl"), tol)
    doc.removeObject(tmp.Name)
doc.save()

bb = scaled(whole).BoundBox
print(f"OK {TRUCK_LENGTH:g} mm build in {OUT_DIR}: size {bb.XLength:.1f} x {bb.YLength:.1f} x {bb.ZLength:.1f} mm; "
      f"valid={whole.isValid()} solids={len(whole.Solids)} "
      f"cab={cab.isValid()} box={cargo.isValid()} chassis={running_gear.isValid()} "
      f"red={all_red.isValid()}/{len(all_red.Solids)} black_text={all_black_text.isValid()}/{len(all_black_text.Solids)}")
print(f"printability (strokes under {MIN_FEATURE * SCALE:.2f} mm): " +
      ", ".join(f"{t} {thin}/{n}" + (f" ({unc} unchecked)" if unc else "") for t, n, thin, unc in thin_report))


# ---- Kit: single-colour pieces that glue together (for batch printing) -----------
#   chassis  black                prints upside down on the flat box seat (tree supports, build plate only)
#   cab      white                prints upright
#   glass    black                windscreen x1 + side window x2, flat; press into the cab's pockets
#   box      white + red + black + grey  prints lying on its side: artwork only in the first/last layers
#   door     white + red + black  rear door panel, prints flat with the outer face up
#   reefer   white + black        fridge unit, prints front face up (black grille in the top layers)
#   wheels   black + silver       print 4 front + 4 rear, outer face up (silver rim in the top layers);
#                                 they push onto axle stubs on the chassis
#   roof     white + red + black  roof artwork panel (incl. QR code), prints face up (colour in the top layers)
# Each piece is exported already in its print orientation (and at SCALE); the colour
# files of one piece share coordinates, so load them together as one multi-part object.
KIT_DIR = os.path.join(OUT_DIR, "kit")
os.makedirs(KIT_DIR, exist_ok=True)
CLEAR = 0.15 / SCALE          # 0.15 mm fit clearance per side for pegs, pins and the frame channel
PEG_R, PEG_H = 1.0, 1.5       # pegs on the box floor; holes in the chassis
HOLE_D = PEG_H + 0.3          # hole depth in the chassis
PIN_R, PIN_L = 0.8, 0.8       # pins on the box locating the door panel
DOOR_SPLIT = BOX_X[0] + 2.0   # the rear door panel is the last 2 mm of the box
BOX_PEGS = [12.0, 62.0]       # x positions on the subframe
DOWEL_R = (1.75 / 2 + 0.1) / SCALE   # holes for dowels cut from 1.75 mm filament
DOWEL_D = 2.3                 # hole depth in each part (dowel length ~2 x this x SCALE, minus a little)
DOOR_PINS = [(-7.0, 13.5), (7.0, 13.5)]   # (y, z), clear of the rear artwork


def cyl(r, h, base, direction):
    return Part.makeCylinder(r, h, base, direction)


up = V(0, 0, 1)
rear_zone = box(-5, DOOR_SPLIT, -20, 20, 0, 50)

# The chassis top stays flat (no pegs) so it can print upside down on it
kit_chassis = running_gear.cut(box(-1, BOX_X[1] + 1, -20, 20, BOX_Z[0], 50))   # stop at the box floor
kit_chassis = kit_chassis.cut(fuse_all(
    [cyl(PEG_R + CLEAR, HOLE_D + 0.1, V(x, 0, BOX_Z[0] - HOLE_D), up) for x in BOX_PEGS] +
    [cyl(DOWEL_R, DOWEL_D + 0.1, V(x, y, CAB_SPLIT - DOWEL_D), up) for x, y in CAB_DOWELS])).removeSplitter()
# Wheels come off as separate pieces; the chassis keeps an axle stub for each to push onto
WHEEL_BORE = 2.2              # stub length inside the wheel
wheel_space = fuse_all([cyl_y(x, AXLE_Z, WHEEL_R + 0.02, WHEEL_Y - 0.02, WHEEL_Y + w + 1) for x, w in AXLES])
stubs = fuse_all([cyl_y(x, AXLE_Z, 1.0, WHEEL_Y - 0.1, WHEEL_Y + WHEEL_BORE - 0.2) for x, _ in AXLES])
kit_chassis = kit_chassis.cut(wheel_space.fuse(mirror_y(wheel_space))).fuse(stubs.fuse(mirror_y(stubs)))
kit_chassis = kit_chassis.removeSplitter()
kit_wheels = {}
for w, name in ((FRONT_WHEEL_W, "front"), (REAR_WHEEL_W, "rear")):
    x, tyre, rim = wheel_protos[w]
    if w < REAR_WHEEL_W:
        # full-diameter spacer on the inner face: makes the front wheels as tall as the rear
        # ones when printed face up, so all the silver layers line up (fewer colour changes),
        # and gives them a full flat base on the plate (a narrow collar here left them standing
        # on ~12 mm2 and they came loose). Hidden against the chassis; the wheel sits where it did.
        tyre = tyre.fuse(cyl_y(x, AXLE_Z, WHEEL_R, WHEEL_Y - (REAR_WHEEL_W - w), WHEEL_Y + 0.05))
    bore = cyl_y(x, AXLE_Z, 1.0 + CLEAR, WHEEL_Y - 1, WHEEL_Y + WHEEL_BORE)
    kit_wheels[name] = (tyre.cut(bore).removeSplitter(), rim)

# The white cab sits flat on the black lower cab; two filament dowels locate it
kit_cab = cab.cut(fuse_all([cyl(DOWEL_R, DOWEL_D + 0.1, V(x, y, CAB_SPLIT - 0.1), up) for x, y in CAB_DOWELS]))
# window pockets with fit clearance for the separate glass panels
side_pocket = win_face.makeOffset2D(GLASS_CLEAR, join=0).extrude(V(0, GLASS_T + 1.02, 0))
side_pocket.translate(V(0, HW - GLASS_T - 0.02, 0))
kit_cab = kit_cab.cut(ws_panel(GLASS_CLEAR, 0.05, GLASS_T + 0.02)).cut(side_pocket).cut(mirror_y(side_pocket))
kit_cab = kit_cab.removeSplitter()

kit_box = cargo.cut(rear_zone).cut(reefer_envelope)           # leaves the reefer's locating recess
kit_box = kit_box.cut(roof_slab(CLEAR, BOX_Z[1] - ROOF_PANEL_T - 0.02, BOX_Z[1] + 1))  # roof panel recess
kit_roof = roof_panel.cut(roof_red).cut(roof_black).removeSplitter()
kit_box = kit_box.fuse([cyl(PIN_R, PIN_L, V(DOOR_SPLIT, y, z), V(-1, 0, 0)) for y, z in DOOR_PINS])
kit_box = kit_box.fuse([cyl(PEG_R, PEG_H + 0.05, V(x, 0, BOX_Z[0] - PEG_H), up) for x in BOX_PEGS])  # floor pegs
kit_box = kit_box.removeSplitter()
kit_box_red = red_parts.cut(rear_zone).removeSplitter()
kit_box_black = black_text.cut(rear_zone).removeSplitter()
kit_box_grey = grey_parts.cut(rear_zone).removeSplitter()

kit_door = cargo.common(rear_zone)
kit_door = kit_door.cut(fuse_all([cyl(PIN_R + CLEAR, PIN_L + 0.1, V(DOOR_SPLIT + 0.1, y, z), V(-1, 0, 0))
                                  for y, z in DOOR_PINS])).removeSplitter()
kit_door_red = red_parts.common(rear_zone).removeSplitter()
kit_door_black = black_text.common(rear_zone).removeSplitter()

# Sanity check: in assembled position no two pieces may overlap
kit_pieces = {"chassis": kit_chassis, "cab": kit_cab, "glass_ws": glass_ws, "glass_side": glass_side,
              "box": kit_box, "box_red": kit_box_red, "box_black": kit_box_black, "box_grey": kit_box_grey,
              "door": kit_door, "door_red": kit_door_red, "door_black": kit_door_black,
              "reefer": reefer, "reefer_black": reefer_black,
              "wheel_front": kit_wheels["front"][0], "wheel_front_silver": kit_wheels["front"][1],
              "wheel_rear": kit_wheels["rear"][0], "wheel_rear_silver": kit_wheels["rear"][1],
              "roof": kit_roof, "roof_red": roof_red, "roof_black": roof_black}
names = list(kit_pieces)
worst, pair = max((kit_pieces[a].common(kit_pieces[b]).Volume, f"{a}/{b}")
                  for i, a in enumerate(names) for b in names[i + 1:])
print(f"kit: max overlap between pieces {worst * SCALE ** 3:.4f} mm3 ({pair}); " +
      ", ".join(f"{n}={'ok' if s.isValid() else 'INVALID'}/{len(s.Solids)}" for n, s in kit_pieces.items()))


def to_print(shapes, axis=None, angle=0.0):
    """Rotate a piece (and its colour partner) into print orientation, sitting on z=0."""
    out = [scaled(s) for s in shapes]
    if axis is not None:
        for s in out:
            s.rotate(V(0, 0, 0), axis, angle)
    bb = out[0].BoundBox
    for s in out:
        s.translate(V(-bb.XMin, -bb.YMin, -bb.ZMin))
    return out


chassis_p, = to_print([kit_chassis], V(1, 0, 0), 180)                                    # upside down
cab_p, = to_print([kit_cab])
glass_ws_p, = to_print([glass_ws], V(0, 1, 0), -math.degrees(math.atan2(ws_n.x, ws_n.z)))   # outer face up
glass_side_p, = to_print([glass_side], V(1, 0, 0), 90)                                     # outer face up
box_p, box_red_p, box_black_p, box_grey_p = to_print([kit_box, kit_box_red, kit_box_black, kit_box_grey],
                                                     V(1, 0, 0), 90)                              # right side down
door_p, door_red_p, door_black_p = to_print([kit_door, kit_door_red, kit_door_black], V(0, 1, 0), 90)  # face up
reefer_p, reefer_black_p = to_print([reefer, reefer_black], V(0, 1, 0), -90)                         # front face up
wf_p, wf_silver_p = to_print(list(kit_wheels["front"]), V(1, 0, 0), 90)                              # outer face up
wr_p, wr_silver_p = to_print(list(kit_wheels["rear"]), V(1, 0, 0), 90)
roof_p, roof_red_p, roof_black_p = to_print([kit_roof, roof_red, roof_black])                       # face up as is

kit_doc = App.newDocument("ReeferTruckKit")
layout = [("Kit_chassis_black", chassis_p, 0),
          ("Kit_cab_white", cab_p, 50),
          ("Kit_glass_windscreen_black", glass_ws_p, 285), ("Kit_glass_side_black", glass_side_p, 300),
          ("Kit_box_white", box_p, 105), ("Kit_box_red", box_red_p, 105), ("Kit_box_black", box_black_p, 105),
          ("Kit_box_grey", box_grey_p, 105),
          ("Kit_door_white", door_p, 155), ("Kit_door_red", door_red_p, 155),
          ("Kit_door_black", door_black_p, 155),
          ("Kit_reefer_white", reefer_p, 200), ("Kit_reefer_black", reefer_black_p, 200),
          ("Kit_wheel_front_black", wf_p, 230), ("Kit_wheel_front_silver", wf_silver_p, 230),
          ("Kit_wheel_rear_black", wr_p, 250), ("Kit_wheel_rear_silver", wr_silver_p, 250),
          ("Kit_roof_white", roof_p, 270), ("Kit_roof_red", roof_red_p, 270), ("Kit_roof_black", roof_black_p, 270)]
layout = [entry for entry in layout if entry[1].Solids]
for name, shp, y_off in layout:
    shp_path = os.path.join(KIT_DIR, f"{name[4:]}.stl")
    tmp = kit_doc.addObject("Part::Feature", name + "_export")
    tmp.Shape = shp
    Mesh.export([tmp], shp_path, 0.02 if "white" in name or "chassis" in name else 0.01)
    kit_doc.removeObject(tmp.Name)
    shown = shp.copy()
    shown.translate(V(0, y_off, 0))        # side-by-side preview layout in the .FCStd
    kit_doc.addObject("Part::Feature", name).Shape = shown
kit_doc.recompute()
kit_doc.saveAs(os.path.join(OUT_DIR, "ReeferTruck_kit.FCStd"))

# Ready-made plate of all 8 wheels (4 front, 4 rear), black and silver files sharing coordinates
plate = {"black": Mesh.Mesh(), "silver": Mesh.Mesh()}
for row, kind in enumerate(("front", "rear")):
    blk = Mesh.Mesh(os.path.join(KIT_DIR, f"wheel_{kind}_black.stl"))
    sil = Mesh.Mesh(os.path.join(KIT_DIR, f"wheel_{kind}_silver.stl"))
    pitch = blk.BoundBox.XLength + 4.0
    for i in range(4):
        for src, key in ((blk, "black"), (sil, "silver")):
            m = src.copy()
            m.translate(i * pitch, row * pitch, 0)
            plate[key].addMesh(m)
for key, m in plate.items():
    m.write(os.path.join(KIT_DIR, f"wheels_plate_{key}.stl"))

# Ready-made plate of one truck's glass: windscreen + left and right side windows, all outer
# face up so each shows its smooth top surface (the right window is the left one mirrored)
glass_right_p, = to_print([mirror_y(glass_side)], V(1, 0, 0), -90)
glass_parts, x = [], 0.0
for piece in (glass_ws_p, glass_side_p, glass_right_p):
    q = piece.copy()
    q.translate(V(x - q.BoundBox.XMin, -q.BoundBox.YMin, 0))
    glass_parts.append(q)
    x += piece.BoundBox.XLength + 4.0
tmp = kit_doc.addObject("Part::Feature", "GlassPlate")
tmp.Shape = Part.makeCompound(glass_parts)
Mesh.export([tmp], os.path.join(KIT_DIR, "glass_plate_black.stl"), 0.01)
kit_doc.removeObject(tmp.Name)
for name, shp, _ in layout:
    b = shp.BoundBox
    print(f"  {name[4:]:14s} {b.XLength:5.1f} x {b.YLength:5.1f} x {b.ZLength:5.1f} mm (print height {b.ZLength:.1f})")
