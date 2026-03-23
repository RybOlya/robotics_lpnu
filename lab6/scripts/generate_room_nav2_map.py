#!/usr/bin/env python3
"""Regenerate lab6/maps/room_nav2.pgm from obstacle geometry (must match room_nav2.sdf)."""
import os

RES = 0.05
ORIGIN_X, ORIGIN_Y = -4.0, -4.0
W = H = 160


def occupied(wx: float, wy: float) -> bool:
    if -4.1 <= wx <= 4.1 and 3.95 <= wy <= 4.05:
        return True
    if -4.1 <= wx <= 4.1 and -4.05 <= wy <= -3.95:
        return True
    if -4.05 <= wx <= -3.95 and -4.1 <= wy <= 4.1:
        return True
    if 3.95 <= wx <= 4.05 and -4.1 <= wy <= 4.1:
        return True
    if 1.025 <= wx <= 1.375 and 0.325 <= wy <= 0.675:
        return True
    if -1.75 <= wx <= -1.25 and -1.25 <= wy <= -0.75:
        return True
    if -0.6 <= wx <= 0.6 and 1.925 <= wy <= 2.075:
        return True
    # nav2_obstacle_pillar_b: pose (2.4, -1.8), size 0.35 x 0.35
    if 2.225 <= wx <= 2.575 and -1.975 <= wy <= -1.625:
        return True
    # nav2_obstacle_block_b: pose (-2.5, 1.5), size 0.45 x 0.45
    if -2.725 <= wx <= -2.275 and 1.275 <= wy <= 1.725:
        return True
    # nav2_obstacle_bar_b: pose (-1.0, -2.5), size 0.8 x 0.12
    if -1.4 <= wx <= -0.6 and -2.56 <= wy <= -2.44:
        return True
    # nav2_obstacle_pillar_c: pose (2.0, 2.5), size 0.35 x 0.35
    if 1.825 <= wx <= 2.175 and 2.325 <= wy <= 2.675:
        return True
    # nav2_obstacle_block_c: pose (-2.0, -2.2), size 0.4 x 0.4
    if -2.2 <= wx <= -1.8 and -2.4 <= wy <= -2.0:
        return True
    # nav2_obstacle_pillar_d: pose (1.5, -0.5), size 0.35 x 0.35
    if 1.325 <= wx <= 1.675 and -0.675 <= wy <= -0.325:
        return True
    # nav2_obstacle_bar_c: pose (-2.8, -0.5), size 0.5 x 0.12
    if -3.05 <= wx <= -2.55 and -0.56 <= wy <= -0.44:
        return True
    return False


def main() -> None:
    root = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
    path = os.path.join(root, "maps", "room_nav2.pgm")
    os.makedirs(os.path.dirname(path), exist_ok=True)
    rows = []
    for iy in range(H):
        wy = ORIGIN_Y + (iy + 0.5) * RES
        row = []
        for ix in range(W):
            wx = ORIGIN_X + (ix + 0.5) * RES
            row.append(0 if occupied(wx, wy) else 254)
        # Mirror across map Y axis (world x -> -x) so PGM columns match map_server/RViz + Gazebo.
        rows.append(row[::-1])
    with open(path, "w", encoding="ascii") as f:
        f.write(f"P2\n{W} {H}\n255\n")
        for row in rows:
            f.write(" ".join(str(v) for v in row) + "\n")
    print("Wrote", path)


if __name__ == "__main__":
    main()
