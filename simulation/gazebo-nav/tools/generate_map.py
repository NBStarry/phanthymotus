#!/usr/bin/env python3
"""Generate the deterministic static map used by the matching Gazebo world."""

from pathlib import Path
import sys

WIDTH = 100
HEIGHT = 80
RESOLUTION = 0.1
ORIGIN_X = -5.0
ORIGIN_Y = -4.0


def _occupied(x: float, y: float) -> bool:
    if x < -4.8 or x > 4.8 or y < -3.8 or y > 3.8:
        return True
    if 1.2 <= x <= 1.8 and 0.0 <= y <= 2.0:
        return True
    if -1.9 <= x <= -1.1 and -0.9 <= y <= -0.1:
        return True
    return False


def generate(path: Path) -> None:
    rows = []
    for image_row in range(HEIGHT):
        map_row = HEIGHT - 1 - image_row
        values = []
        for col in range(WIDTH):
            x = ORIGIN_X + (col + 0.5) * RESOLUTION
            y = ORIGIN_Y + (map_row + 0.5) * RESOLUTION
            values.append("0" if _occupied(x, y) else "254")
        rows.append(" ".join(values))
    path.parent.mkdir(parents=True, exist_ok=True)
    path.write_text(f"P2\n{WIDTH} {HEIGHT}\n255\n" + "\n".join(rows) + "\n", encoding="ascii")


if __name__ == "__main__":
    if len(sys.argv) > 2:
        raise SystemExit(f"usage: {sys.argv[0]} [OUTPUT_PATH]")
    output = (
        Path(sys.argv[1])
        if len(sys.argv) == 2
        else Path(__file__).resolve().parents[1] / "maps" / "synthetic_room.pgm"
    )
    generate(output)
