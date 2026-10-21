"""Convert Quisk's truck artwork SVGs into aldom_final_artwork.json for reefer_truck.py.

Run:  python quisk_svg.py        (standard library only; any Python 3)

Input: the Illustrator SVG exports in Aldom-3D-Truck_Artwork/SVG, one panel per file, with one
layer per colour (text already in outlines). PANELS below says which file is used for each panel.

Output, per panel (units are the SVG's, i.e. the artwork's own millimetres):
  size           [width, height] from the viewBox
  red/black/grey lists of fills {"colour", "subpaths": [[seg, ...], ...]}; a seg is
                 ["L", p0, p1] or ["C", p0, c1, c2, p1]; points are [x, v] with v UP from the
                 panel's bottom edge (SVG y runs down)
  rounded_white  white rounded rectangles (the panel behind the roof QR code): {x, v, w, h, r}
  qr             roof only: where the QR code's grid sits {x, v (top edge), pitch, modules}. The
                 squares themselves come from aldom_qr.json, so they are not stored here.
White shapes are the background and are otherwise ignored.
"""
import json
import math
import os
import re
import statistics
import xml.etree.ElementTree as ET

HERE = os.path.dirname(os.path.abspath(__file__))
SVG_DIR = os.path.join(HERE, "Aldom-3D-Truck_Artwork", "SVG")
OUT = os.path.join(HERE, "aldom_final_artwork.json")
PANELS = {
    "side_left": "1_Truck-Side_L.svg",
    "side_right": "2_Truck-Side_R.svg",
    "back": "3_Truck-Back.svg",
    "top": "4_Truck-Top-Revised.svg",
}
COLOURS = {"#be1320": "red", "#231f20": "black", "#010101": "black", "#000000": "black",
           "#bfbfbf": "grey", "#ffffff": "white"}
CLOSE_SNAP = 0.2              # a path ending this close to its start is snapped shut (the tagline's G's
                              # overshoot their start by ~0.1, which would make a tiny self-crossing)
KAPPA = 0.5522847498          # cubic Bezier handle length for a quarter circle
NS = "{http://www.w3.org/2000/svg}"


# ---- small geometry helpers ---------------------------------------------------------------
def mat_mul(a, b):
    """Affine matrices as (a, b, c, d, e, f) like SVG's matrix(); returns a then b applied to a point = a*b."""
    return (a[0] * b[0] + a[2] * b[1], a[1] * b[0] + a[3] * b[1],
            a[0] * b[2] + a[2] * b[3], a[1] * b[2] + a[3] * b[3],
            a[0] * b[4] + a[2] * b[5] + a[4], a[1] * b[4] + a[3] * b[5] + a[5])


IDENTITY = (1, 0, 0, 1, 0, 0)


def parse_transform(text):
    m = IDENTITY
    for name, args in re.findall(r"(\w+)\s*\(([^)]*)\)", text or ""):
        v = [float(x) for x in re.findall(r"[-+]?(?:\d+\.?\d*|\.\d+)(?:[eE][-+]?\d+)?", args)]
        if name == "translate":
            t = (1, 0, 0, 1, v[0], v[1] if len(v) > 1 else 0)
        elif name == "scale":
            t = (v[0], 0, 0, v[1] if len(v) > 1 else v[0], 0, 0)
        elif name == "rotate":
            a = math.radians(v[0])
            r = (math.cos(a), math.sin(a), -math.sin(a), math.cos(a), 0, 0)
            if len(v) == 3:                                     # rotate about (cx, cy)
                r = mat_mul(mat_mul((1, 0, 0, 1, v[1], v[2]), r), (1, 0, 0, 1, -v[1], -v[2]))
            t = r
        elif name == "matrix":
            t = tuple(v)
        elif name == "skewX":
            t = (1, 0, math.tan(math.radians(v[0])), 1, 0, 0)
        elif name == "skewY":
            t = (1, math.tan(math.radians(v[0])), 0, 1, 0, 0)
        else:
            raise ValueError(f"unsupported transform {name}")
        m = mat_mul(m, t)
    return m


def apply(m, p):
    return (m[0] * p[0] + m[2] * p[1] + m[4], m[1] * p[0] + m[3] * p[1] + m[5])


# ---- SVG path data -> subpaths of ("L", p0, p1) / ("C", p0, c1, c2, p1) in SVG coordinates --
TOKEN = re.compile(r"[MmLlHhVvCcSsQqTtAaZz]|[-+]?(?:\d+\.?\d*|\.\d+)(?:[eE][-+]?\d+)?")


def arc_to_cubics(p0, rx, ry, phi, large, sweep, p1):
    """SVG elliptical arc as cubic Beziers (W3C implementation notes, F.6)."""
    if p0 == p1:
        return []
    if rx == 0 or ry == 0:
        return [("L", p0, p1)]
    rx, ry = abs(rx), abs(ry)
    cp, sp = math.cos(math.radians(phi)), math.sin(math.radians(phi))
    dx, dy = (p0[0] - p1[0]) / 2, (p0[1] - p1[1]) / 2
    x1, y1 = cp * dx + sp * dy, -sp * dx + cp * dy
    lam = x1 * x1 / (rx * rx) + y1 * y1 / (ry * ry)
    if lam > 1:
        rx, ry = rx * math.sqrt(lam), ry * math.sqrt(lam)
    num = rx * rx * ry * ry - rx * rx * y1 * y1 - ry * ry * x1 * x1
    den = rx * rx * y1 * y1 + ry * ry * x1 * x1
    co = math.sqrt(max(0.0, num / den)) * (-1 if large == sweep else 1)
    cx1, cy1 = co * rx * y1 / ry, -co * ry * x1 / rx
    cx, cy = cp * cx1 - sp * cy1 + (p0[0] + p1[0]) / 2, sp * cx1 + cp * cy1 + (p0[1] + p1[1]) / 2

    def ang(ux, uy, vx, vy):
        a = math.atan2(ux * vy - uy * vx, ux * vx + uy * vy)
        return a
    t1 = ang(1, 0, (x1 - cx1) / rx, (y1 - cy1) / ry)
    dt = ang((x1 - cx1) / rx, (y1 - cy1) / ry, (-x1 - cx1) / rx, (-y1 - cy1) / ry)
    if not sweep and dt > 0:
        dt -= 2 * math.pi
    elif sweep and dt < 0:
        dt += 2 * math.pi
    n = max(1, math.ceil(abs(dt) / (math.pi / 2)))
    d = dt / n
    k = 4 / 3 * math.tan(d / 4)
    segs, start = [], p0

    def pt(t):
        return (cx + rx * math.cos(t) * cp - ry * math.sin(t) * sp, cy + rx * math.cos(t) * sp + ry * math.sin(t) * cp)

    def deriv(t):
        return (-rx * math.sin(t) * cp - ry * math.cos(t) * sp, -rx * math.sin(t) * sp + ry * math.cos(t) * cp)
    for i in range(n):
        a0, a1 = t1 + i * d, t1 + (i + 1) * d
        e = p1 if i == n - 1 else pt(a1)
        d0, d1 = deriv(a0), deriv(a1)
        segs.append(("C", start, (start[0] + k * d0[0], start[1] + k * d0[1]), (e[0] - k * d1[0], e[1] - k * d1[1]), e))
        start = e
    return segs


def parse_path(d):
    toks = TOKEN.findall(d)
    i, cmd = 0, None
    cur = start = (0.0, 0.0)
    last_ctrl, last_cmd = None, None
    subpaths, segs = [], []

    def num():
        nonlocal i
        v = float(toks[i])
        i += 1
        return v

    def flag():                       # arc flags may be written without separators ("0 01" or "011.5")
        nonlocal i
        t = toks[i]
        if len(t) > 1 and t[0] in "01":
            toks[i] = t[1:]           # the rest of the token is the next flag or number
            return int(t[0])
        i += 1
        return int(float(t))

    def finish():
        if segs:
            subpaths.append(list(segs))
        segs.clear()

    while i < len(toks):
        if toks[i].isalpha():
            cmd = toks[i]
            i += 1
        elif cmd is None:
            raise ValueError("path data does not start with a command")
        rel = cmd.islower()
        c = cmd.upper()
        base = cur if rel else (0.0, 0.0)
        if c == "Z":
            if segs and 0 < math.dist(cur, start) < CLOSE_SNAP:
                # ends just short of / just past its start (the tagline's G's overshoot by 0.1): move
                # the last point onto the start rather than adding a tiny closing edge, which would
                # leave a zero-width spike and split the letter in two
                segs[-1] = snap_end(segs[-1], start)
            elif cur != start:
                segs.append(("L", cur, start))
            cur = start
            finish()
            last_ctrl, last_cmd = None, "Z"
            continue
        if c == "M":
            finish()
            cur = start = (base[0] + num(), base[1] + num())
            cmd = "l" if rel else "L"                      # further pairs are line-tos
            last_ctrl, last_cmd = None, "M"
            continue
        if c in "LHV":
            if c == "L":
                p = (base[0] + num(), base[1] + num())
            elif c == "H":
                p = ((cur[0] if rel else 0.0) + num(), cur[1])
            else:
                p = (cur[0], (cur[1] if rel else 0.0) + num())
            segs.append(("L", cur, p))
            cur, last_ctrl, last_cmd = p, None, c
        elif c in "CS":
            if c == "C":
                c1 = (base[0] + num(), base[1] + num())
            else:                                           # reflect the previous curve's handle
                c1 = (2 * cur[0] - last_ctrl[0], 2 * cur[1] - last_ctrl[1]) if last_cmd in "CS" and last_ctrl else cur
            c2 = (base[0] + num(), base[1] + num())
            p = (base[0] + num(), base[1] + num())
            segs.append(("C", cur, c1, c2, p))
            cur, last_ctrl, last_cmd = p, c2, c
        elif c in "QT":
            if c == "Q":
                q = (base[0] + num(), base[1] + num())
            else:
                q = (2 * cur[0] - last_ctrl[0], 2 * cur[1] - last_ctrl[1]) if last_cmd in "QT" and last_ctrl else cur
            p = (base[0] + num(), base[1] + num())
            c1 = (cur[0] + 2 / 3 * (q[0] - cur[0]), cur[1] + 2 / 3 * (q[1] - cur[1]))
            c2 = (p[0] + 2 / 3 * (q[0] - p[0]), p[1] + 2 / 3 * (q[1] - p[1]))
            segs.append(("C", cur, c1, c2, p))
            cur, last_ctrl, last_cmd = p, q, c
        elif c == "A":
            rx, ry, phi = num(), num(), num()
            large, sweep = flag(), flag()
            p = (base[0] + num(), base[1] + num())
            segs.extend(arc_to_cubics(cur, rx, ry, phi, large, sweep, p))
            cur, last_ctrl, last_cmd = p, None, c
    finish()
    return subpaths


def snap_end(seg, target):
    """Move a segment's end point onto `target`, taking a curve's end handle with it (moving the
    point alone would leave the curve overshooting and doubling back - a zero-width spike)."""
    d = (target[0] - seg[-1][0], target[1] - seg[-1][1])
    if seg[0] == "C":
        return ("C", seg[1], seg[2], (seg[3][0] + d[0], seg[3][1] + d[1]), target)
    return (seg[0], seg[1], target)


def rect_subpath(x, y, w, h, rx, ry):
    if rx <= 0 and ry <= 0:
        p = [(x, y), (x + w, y), (x + w, y + h), (x, y + h)]
        return [[("L", p[i], p[(i + 1) % 4]) for i in range(4)]]
    rx, ry = min(rx or ry, w / 2), min(ry or rx, h / 2)
    kx, ky = rx * KAPPA, ry * KAPPA
    s = []
    s.append(("L", (x + rx, y), (x + w - rx, y)))
    s.append(("C", (x + w - rx, y), (x + w - rx + kx, y), (x + w, y + ry - ky), (x + w, y + ry)))
    s.append(("L", (x + w, y + ry), (x + w, y + h - ry)))
    s.append(("C", (x + w, y + h - ry), (x + w, y + h - ry + ky), (x + w - rx + kx, y + h), (x + w - rx, y + h)))
    s.append(("L", (x + w - rx, y + h), (x + rx, y + h)))
    s.append(("C", (x + rx, y + h), (x + rx - kx, y + h), (x, y + h - ry + ky), (x, y + h - ry)))
    s.append(("L", (x, y + h - ry), (x, y + ry)))
    s.append(("C", (x, y + ry), (x, y + ry - ky), (x + rx - kx, y), (x + rx, y)))
    return [s]


# ---- one SVG file -> panel dict ------------------------------------------------------------------
def norm_colour(c):
    c = c.strip().lower()
    if re.fullmatch(r"#[0-9a-f]{3}", c):
        c = "#" + "".join(ch * 2 for ch in c[1:])
    return c


def convert(path):
    root = ET.parse(path).getroot()
    vb = [float(v) for v in re.split(r"[\s,]+", root.get("viewBox").strip())]
    W, H = vb[2], vb[3]
    css = {}
    for style in root.iter(NS + "style"):
        for cls, body in re.findall(r"\.([\w-]+)\s*\{([^}]*)\}", style.text or ""):
            m = re.search(r"fill\s*:\s*([^;]+)", body)
            if m:
                css[cls] = norm_colour(m.group(1))

    def fill_of(el, inherited):
        if el.get("fill"):
            return norm_colour(el.get("fill"))
        m = re.search(r"fill\s*:\s*([^;]+)", el.get("style", ""))
        if m:
            return norm_colour(m.group(1))
        for cls in (el.get("class") or "").split():
            if cls in css:
                return css[cls]
        return inherited

    out = {"size": [W, H], "red": [], "black": [], "grey": [], "rounded_white": []}
    qr_rects = []
    unknown = set()

    def to_panel(p):                                        # SVG y-down -> panel v-up, viewBox origin
        return [round(p[0] - vb[0], 4), round(H - (p[1] - vb[1]), 4)]

    def walk(el, m, inherited):
        m = mat_mul(m, parse_transform(el.get("transform")))
        fill = fill_of(el, inherited)
        tag = el.tag.replace(NS, "")
        subpaths = None
        if tag in ("g", "svg"):
            for child in el:
                walk(child, m, fill)
            return
        if tag == "path":
            subpaths = parse_path(el.get("d", ""))
        elif tag in ("polygon", "polyline"):
            v = [float(x) for x in TOKEN.findall(el.get("points", ""))]
            pts = list(zip(v[0::2], v[1::2]))
            subpaths = [[("L", pts[i], pts[i + 1]) for i in range(len(pts) - 1)]]
        elif tag == "rect":
            x, y = float(el.get("x", 0)), float(el.get("y", 0))
            w, h = float(el.get("width")), float(el.get("height"))
            rx, ry = float(el.get("rx", 0) or 0), float(el.get("ry", 0) or 0)
            colour = COLOURS.get(fill)
            if colour == "white":
                if rx or ry:
                    a, b = apply(m, (x, y)), apply(m, (x + w, y + h))
                    out["rounded_white"].append({"x": round(min(a[0], b[0]) - vb[0], 4),
                                                 "v": round(H - (max(a[1], b[1]) - vb[1]), 4),
                                                 "w": round(abs(b[0] - a[0]), 4), "h": round(abs(b[1] - a[1]), 4),
                                                 "r": round(rx or ry, 4)})
                return
            if colour == "black" and not (rx or ry) and w <= 3 and h <= 3:
                a = apply(m, (x, y))
                qr_rects.append((a[0], a[1], w, h))         # candidate QR module
                return
            subpaths = rect_subpath(x, y, w, h, rx, ry)
        elif tag in ("circle", "ellipse"):
            cx, cy = float(el.get("cx", 0)), float(el.get("cy", 0))
            rx = float(el.get("r", el.get("rx", 0)))
            ry = float(el.get("r", el.get("ry", 0)))
            subpaths = rect_subpath(cx - rx, cy - ry, 2 * rx, 2 * ry, rx, ry)
        else:
            return                                          # defs, style, title, metadata ...
        colour = COLOURS.get(fill)
        if colour is None:
            unknown.add(fill)
            return
        if colour == "white":
            return
        fill_out = []
        for sp in subpaths:
            sp = [tuple([s[0]] + [apply(m, q) for q in s[1:]]) for s in sp]
            sp = [s for s in sp if any(abs(s[k][0] - s[1][0]) > 1e-9 or abs(s[k][1] - s[1][1]) > 1e-9
                                       for k in range(2, len(s)))]           # drop zero-length segments
            if not sp:
                continue
            start, end = sp[0][1], sp[-1][-1]
            gap = math.dist(start, end)
            if 0 < gap < CLOSE_SNAP:
                sp[-1] = snap_end(sp[-1], start)                           # snap shut
            elif gap >= CLOSE_SNAP:
                sp.append(("L", end, start))
            fill_out.append([[s[0]] + [to_panel(q) for q in s[1:]] for s in sp])
        if fill_out:
            out[colour].append({"colour": colour, "subpaths": fill_out})

    walk(root, IDENTITY, None)
    if unknown:
        raise ValueError(f"{os.path.basename(path)}: unknown fill colours {sorted(unknown)} - add them to COLOURS")

    # A block of many small black squares is the QR code: keep where its grid sits
    if len(qr_rects) >= 50:
        # The squares are drawn a little larger than the grid (they overlap) and their positions
        # are rounded to 0.1, so measure the grid from the first column to the last.
        xs = sorted({round(r[0], 3) for r in qr_rects})
        steps = [b - a for a, b in zip(xs, xs[1:]) if b - a > 0.3]
        typical = statistics.median(steps)
        est = statistics.mean([s for s in steps if s < 1.5 * typical])      # neighbouring columns only
        modules = round((xs[-1] - xs[0]) / est) + 1
        y0 = min(r[1] for r in qr_rects)
        out["qr"] = {"x": round(xs[0] - vb[0], 4), "v": round(H - (y0 - vb[1]), 4),
                     "pitch": (xs[-1] - xs[0]) / (modules - 1), "modules": modules}
    elif qr_rects:                                          # just a few small squares: ordinary artwork
        for x, y, w, h in qr_rects:
            sp = rect_subpath(x, y, w, h, 0, 0)[0]
            out["black"].append({"colour": "black", "subpaths": [[[s[0]] + [to_panel(q) for q in s[1:]] for s in sp]]})
    return out


def main():
    import argparse
    ap = argparse.ArgumentParser(description=__doc__.splitlines()[0])
    ap.add_argument("--out", default=OUT, help="output JSON (default: aldom_final_artwork.json)")
    for panel, name in PANELS.items():
        ap.add_argument(f"--{panel.replace('_', '-')}", default=name, help=f"SVG for the {panel} panel (default: {name})")
    args = ap.parse_args()
    art = {}
    for panel in PANELS:
        name = getattr(args, panel)
        art[panel] = convert(os.path.join(SVG_DIR, name))
        p = art[panel]
        extra = f", QR {p['qr']['modules']}x{p['qr']['modules']}" if "qr" in p else ""
        print(f"{panel:10s} <- {name}: {p['size'][0]:g} x {p['size'][1]:g}, red {len(p['red'])}, "
              f"black {len(p['black'])}, grey {len(p['grey'])}, rounded white {len(p['rounded_white'])}{extra}")
    with open(args.out, "w") as f:
        json.dump(art, f)
    print("wrote", args.out)


if __name__ == "__main__":
    main()
