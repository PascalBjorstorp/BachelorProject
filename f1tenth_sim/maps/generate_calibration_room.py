#!/usr/bin/env python3
"""Generate the purpose-built 14 m square calibration-room occupancy map."""
from __future__ import annotations

from pathlib import Path


PIXELS = 140
FREE = 254
OCCUPIED = 0


def rectangle(image: list[list[int]], x0: int, y0: int, x1: int, y1: int) -> None:
    """Fill an inclusive-exclusive occupied rectangle."""
    for y in range(max(0, y0), min(PIXELS, y1)):
        for x in range(max(0, x0), min(PIXELS, x1)):
            image[y][x] = OCCUPIED


def main() -> int:
    image = [[FREE for _ in range(PIXELS)] for _ in range(PIXELS)]

    # Three-pixel structural walls. All additional fixed features stay inside
    # the 1 m wall-clearance band, leaving pixels [10:130, 10:130] completely
    # open for the physical 12 x 12 m operating square.
    rectangle(image, 0, 0, PIXELS, 3)
    rectangle(image, 0, PIXELS - 3, PIXELS, PIXELS)
    rectangle(image, 0, 0, 3, PIXELS)
    rectangle(image, PIXELS - 3, 0, PIXELS, PIXELS)

    # Deliberately asymmetric wall returns improve point-to-line ICP geometry
    # without placing obstacles in the driving region.
    rectangle(image, 3, 20, 8, 43)
    rectangle(image, 3, 92, 6, 119)
    rectangle(image, 132, 31, 137, 55)
    rectangle(image, 134, 88, 137, 128)
    rectangle(image, 24, 3, 47, 8)
    rectangle(image, 78, 3, 103, 6)
    rectangle(image, 48, 133, 73, 137)
    rectangle(image, 108, 131, 128, 137)

    output = Path(__file__).with_name("calibration_room_map.pgm")
    lines = [
        "P2",
        "# 14.0 x 14.0 m calibration room; central 12 x 12 m is obstacle-free",
        f"{PIXELS} {PIXELS}",
        "255",
        *(" ".join(map(str, row)) for row in image),
    ]
    output.write_text("\n".join(lines) + "\n", encoding="ascii")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
