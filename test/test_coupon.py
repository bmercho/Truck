"""0.2 mm nozzle test coupon for the Aldom truck (real millimetres, single colour).

Run headless:  freecadcmd.exe test_coupon.py
Exports test_coupon.stl (the coupon) and test_ring.stl (a ring for the axle-stub fit test).

What it tests:
  * the finest artwork on the truck: the ALDOM wordmark at cab-door size, the tagline at its
    smallest size (rear doors, ~0.24 mm strokes) and a line of the slogan
  * raised lines 0.22 / 0.3 / 0.4 / 0.6 mm wide
  * fits: a dowel hole for 1.75 mm filament (as used to locate the cab) and an axle stub
    with a separate ring at the wheels' 0.15 mm clearance
"""
import os
import json
import FreeCAD as App
import Part
import Mesh

V = App.Vector
HERE = os.path.dirname(os.path.abspath(__file__))
ARTWORK = json.load(open(os.path.join(HERE, "..", "aldom_artwork.json")))

PLATE = (46.0, 26.0, 1.2)     # x, y, thickness
RAISE = 0.6                   # height of the raised artwork and lines
CLEAR = 0.15                  # fit clearance per side, as in the truck kit


def box(x0, x1, y0, y1, z0, z1):
    return Part.makeBox(x1 - x0, y1 - y0, z1 - z0, V(x0, y0, z0))


def artwork(names, width, u0, v0):
    """Brand artwork scaled to `width`, bottom-left at (u0, v0), as faces in the XY plane."""
    pts = [p for n in names for f in ARTWORK[n] for sp in f["subpaths"] for seg in sp for p in seg[1:]]
    x0, y0 = min(p[0] for p in pts), min(p[1] for p in pts)
    k = width / (max(p[0] for p in pts) - x0)
    faces = []
    for n in names:
        for fill in ARTWORK[n]:
            wires = []
            for sp in fill["subpaths"]:
                edges = []
                for seg in sp:
                    q = [V((p[0] - x0) * k + u0, (p[1] - y0) * k + v0, 0) for p in seg[1:]]
                    if seg[0] == "L":
                        if (q[1] - q[0]).Length > 1e-7:
                            edges.append(Part.LineSegment(q[0], q[1]).toShape())
                    else:
                        bz = Part.BezierCurve()
                        bz.setPoles(q)
                        edges.append(bz.toShape())
                wires.append(Part.Wire(edges))
            faces.append(Part.makeFace(wires, "Part::FaceMakerBullseye"))
    return faces


def text(txt, cap_h, u0, v0):
    wires = [w for ch in Part.makeWireString(txt, "C:/Windows/Fonts/", "arialbd.ttf", 10.0, 0) for w in ch]
    f = Part.makeFace(wires, "Part::FaceMakerBullseye")
    href = Part.makeFace([w for ch in Part.makeWireString("H", "C:/Windows/Fonts/", "arialbd.ttf", 10.0, 0) for w in ch],
                         "Part::FaceMakerBullseye").BoundBox.YLength
    f.scale(cap_h / href)
    f.translate(V(u0 - f.BoundBox.XMin, v0, 0))
    return [f]


def raised(faces):
    solids = []
    for f in faces:
        g = f.copy()
        g.translate(V(0, 0, PLATE[2] - 0.01))
        solids.append(g.extrude(V(0, 0, RAISE + 0.01)))
    return solids


plate = box(0, PLATE[0], 0, PLATE[1], 0, PLATE[2])
plate = plate.makeFillet(1.5, [e for e in plate.Edges if e.BoundBox.ZLength > PLATE[2] - 0.01])

parts = []
# Row 1: wordmark at cab-door size (11.7 mm wide) and the tagline at its smallest size (rear doors)
parts += raised(artwork(["wordmark"], 11.7, 2.0, 20.0))
parts += raised(artwork(["tagline"], 25.5, 17.5, 20.6))
# Row 2: the slogan at its size on the box (2.2 mm capitals)
parts += raised(text("Built on Experience.", 2.18, 2.0, 14.5))
# Row 3: raised lines 0.22 / 0.3 / 0.4 / 0.6 mm wide
x = 27.0
for w in (0.22, 0.3, 0.4, 0.6):
    parts.append(box(x, x + w, 3.0, 11.0, PLATE[2] - 0.01, PLATE[2] + RAISE))
    x += 4.0
# Fit tests: dowel hole for 1.75 mm filament, axle stub for the ring
coupon = plate.fuse(parts)
coupon = coupon.cut(Part.makeCylinder(1.75 / 2 + 0.1, 10, V(6.0, 6.0, -1)))
stub_r, stub_h = 1.5, 3.0                                  # the truck's axle stub (1.0 x 1.5 scale)
coupon = coupon.fuse(Part.makeCylinder(stub_r, stub_h, V(16.0, 6.0, PLATE[2] - 0.01))).removeSplitter()

ring = Part.makeCylinder(3.5, 2.0, V(55.0, 6.0, 0)).cut(Part.makeCylinder(stub_r + CLEAR, 4, V(55.0, 6.0, -1)))

doc = App.newDocument("TestCoupon")
for name, shp in (("test_coupon", coupon), ("test_ring", ring)):
    o = doc.addObject("Part::Feature", name)
    o.Shape = shp
    Mesh.export([o], os.path.join(HERE, name + ".stl"), 0.01)
print(f"OK coupon valid={coupon.isValid()} {coupon.BoundBox.XLength:.1f} x {coupon.BoundBox.YLength:.1f} x "
      f"{coupon.BoundBox.ZLength:.1f} mm; ring valid={ring.isValid()}")
