"""Render A4 test sheets with 2, 4 and 8 ArUco markers per page (vector PDF).

Each sheet uses the largest marker that fits its grid, so the three pages give three
marker sizes for testing at what distance the camera still detects a tag. Every marker is
labelled with its ID and black-square edge length (the value for `marker.size_m`).

The PDF is written by hand as vector rectangles: no extra dependencies, and the printed
size is exact. Print at 100 % / "actual size" (NOT "fit to page") and check the 100 mm
ruler at the bottom of the page with a real ruler.

Usage: python tools/generate_marker_sheets.py [--dict DICT_4X4_50] [--out printables/aruco]
"""

import argparse
from pathlib import Path

import cv2

MM = 72.0 / 25.4  # PDF points per millimetre
PAGE_W_MM, PAGE_H_MM = 210.0, 297.0  # A4

# markers per sheet -> (columns, rows)
LAYOUTS = {2: (1, 2), 4: (2, 2), 8: (2, 4)}

LABEL_H_MM = 7.0  # room under each marker for its caption
FOOTER_H_MM = 18.0  # room at the bottom of the page for the ruler and print note
RULER_MM = 100.0


def marker_modules(dictionary, marker_id):
    """Return the marker as rows of booleans (True = black), including the black border."""
    n = dictionary.markerSize + 2  # data bits + 1-module border on each side
    img = cv2.aruco.generateImageMarker(dictionary, marker_id, n)
    return [[img[r, c] < 128 for c in range(n)] for r in range(n)]


def fit_marker_mm(cols, rows, margin_mm, modules):
    """Largest whole-mm black-square edge that fits a grid cell with quiet zone and label."""
    cell_w = (PAGE_W_MM - 2 * margin_mm) / cols
    cell_h = (PAGE_H_MM - 2 * margin_mm - FOOTER_H_MM) / rows
    # quiet zone of 1 module on every side -> total width = side * (modules + 2) / modules
    k = (modules + 2) / modules
    return int(min(cell_w / k, (cell_h - LABEL_H_MM) / k))


def rect(x_mm, y_mm, w_mm, h_mm):
    """PDF rectangle; (x, y) is the top-left corner in mm measured from the page top."""
    y_pdf = (PAGE_H_MM - y_mm - h_mm) * MM
    return f"{x_mm * MM:.3f} {y_pdf:.3f} {w_mm * MM:.3f} {h_mm * MM:.3f} re f"


def text(x_mm, y_mm, size_pt, s, centered=True):
    """PDF text line; y is the baseline in mm from the page top. Helvetica width estimate."""
    s = s.replace("\\", "\\\\").replace("(", "\\(").replace(")", "\\)")
    if centered:
        x_mm -= len(s) * size_pt * 0.5 / MM / 2
    return f"BT /F1 {size_pt} Tf {x_mm * MM:.3f} {(PAGE_H_MM - y_mm) * MM:.3f} Td ({s}) Tj ET"


def draw_marker(ops, grid, x_mm, y_mm, side_mm):
    """Draw black modules, merging horizontal runs to avoid hairline seams."""
    n = len(grid)
    m = side_mm / n
    for r, row in enumerate(grid):
        c = 0
        while c < n:
            if not row[c]:
                c += 1
                continue
            start = c
            while c < n and row[c]:
                c += 1
            # tiny vertical overlap hides anti-aliasing seams between rows in viewers
            ops.append(rect(x_mm + start * m, y_mm + r * m, (c - start) * m, m + 0.02))


def build_page(dictionary, dict_name, count, margin_mm):
    cols, rows = LAYOUTS[count]
    n = dictionary.markerSize + 2
    side = fit_marker_mm(cols, rows, margin_mm, n)
    quiet = side / n
    cell_w = (PAGE_W_MM - 2 * margin_mm) / cols
    cell_h = (PAGE_H_MM - 2 * margin_mm - FOOTER_H_MM) / rows
    block_h = side + 2 * quiet + LABEL_H_MM

    ops = ["0 g"]  # black fill
    for i in range(count):
        col, row = i % cols, i // cols
        cx = margin_mm + col * cell_w + cell_w / 2
        top = margin_mm + row * cell_h + (cell_h - block_h) / 2
        x, y = cx - side / 2, top + quiet
        draw_marker(ops, marker_modules(dictionary, i), x, y, side)
        ops.append(text(cx, y + side + quiet + 4.5, 10, f"{dict_name}  id {i}  |  {side} mm"))

    # 100 mm ruler with 10 mm ticks, to check that the printer did not scale the page
    ry = PAGE_H_MM - margin_mm - 9
    rx = (PAGE_W_MM - RULER_MM) / 2
    ops.append(rect(rx, ry, RULER_MM, 0.4))
    for t in range(0, int(RULER_MM) + 1, 10):
        ops.append(rect(rx + t - 0.2, ry - 3, 0.4, 3))
    ops.append(text(PAGE_W_MM / 2, ry + 5, 8,
                    f"{RULER_MM:.0f} mm - print at 100% (actual size), measure to verify. "
                    f"Marker edge = {side} mm = size_m {side / 1000:.3f}"))
    return "\n".join(ops), side


def write_pdf(path, content):
    data = content.encode("latin-1")
    objs = [
        b"<< /Type /Catalog /Pages 2 0 R >>",
        b"<< /Type /Pages /Kids [3 0 R] /Count 1 >>",
        (f"<< /Type /Page /Parent 2 0 R /MediaBox [0 0 {PAGE_W_MM * MM:.3f} "
         f"{PAGE_H_MM * MM:.3f}] /Resources << /Font << /F1 5 0 R >> >> "
         f"/Contents 4 0 R >>").encode(),
        b"<< /Length %d >>\nstream\n" % len(data) + data + b"\nendstream",
        b"<< /Type /Font /Subtype /Type1 /BaseFont /Helvetica >>",
    ]
    out = bytearray(b"%PDF-1.4\n")
    offsets = []
    for i, body in enumerate(objs, start=1):
        offsets.append(len(out))
        out += b"%d 0 obj\n" % i + body + b"\nendobj\n"
    xref = len(out)
    out += b"xref\n0 %d\n0000000000 65535 f \n" % (len(objs) + 1)
    for off in offsets:
        out += b"%010d 00000 n \n" % off
    out += b"trailer\n<< /Size %d /Root 1 0 R >>\nstartxref\n%d\n%%%%EOF\n" % (
        len(objs) + 1, xref)
    path.write_bytes(bytes(out))


def main():
    p = argparse.ArgumentParser(description=__doc__,
                                formatter_class=argparse.RawDescriptionHelpFormatter)
    p.add_argument("--dict", default="DICT_4X4_50")
    p.add_argument("--out", type=Path, default=Path("printables/aruco"))
    p.add_argument("--margin-mm", type=float, default=10.0, help="page margin (printer-safe)")
    args = p.parse_args()

    dictionary = cv2.aruco.getPredefinedDictionary(getattr(cv2.aruco, args.dict))
    args.out.mkdir(parents=True, exist_ok=True)
    for count in sorted(LAYOUTS):
        content, side = build_page(dictionary, args.dict, count, args.margin_mm)
        path = args.out / f"a4_{count}x_{args.dict}_{side}mm.pdf"
        write_pdf(path, content)
        print(f"wrote {path}  ({count} markers, {side} mm each)")


if __name__ == "__main__":
    main()
