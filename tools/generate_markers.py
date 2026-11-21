"""Generate printable ArUco marker images.

Usage: python tools/generate_markers.py --ids 0 1 2 3 --dict DICT_4X4_50 --px 600 --out markers/
"""

import argparse
from pathlib import Path

import cv2


def main():
    p = argparse.ArgumentParser(description=__doc__)
    p.add_argument("--ids", type=int, nargs="+", default=[0])
    p.add_argument("--dict", default="DICT_4X4_50")
    p.add_argument("--px", type=int, default=600, help="image side length in pixels")
    p.add_argument("--out", type=Path, default=Path("markers"))
    args = p.parse_args()

    dictionary = cv2.aruco.getPredefinedDictionary(getattr(cv2.aruco, args.dict))
    args.out.mkdir(parents=True, exist_ok=True)
    for marker_id in args.ids:
        img = cv2.aruco.generateImageMarker(dictionary, marker_id, args.px)
        img = cv2.copyMakeBorder(img, 60, 60, 60, 60, cv2.BORDER_CONSTANT, value=255)  # quiet zone
        path = args.out / f"{args.dict}_id{marker_id}.png"
        cv2.imwrite(str(path), img)
        print("wrote", path)


if __name__ == "__main__":
    main()
