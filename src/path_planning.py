from __future__ import annotations

import math
from typing import List, Tuple

try:
    import numpy as np
    from scipy.interpolate import CubicSpline
    _HAS_SCIPY = True
except ImportError:
    _HAS_SCIPY = False

from src.models import CarPose, Cone, Path2D


class PathPlanning:
    """Student-implemented path planner for Formula Student (FSAI)-style track.

    Given the car pose (x, y, yaw) and an array of detected track cones with color flags:
    - color == 0: Yellow cone (Right boundary)
    - color == 1: Blue cone (Left boundary)

    Generates a smooth, drivable path staying between left and right track boundaries.
    """

    def __init__(self, car_pose: CarPose, cones: List[Cone]):
        self.car_pose = car_pose
        self.cones = cones

    def generatePath(self) -> Path2D:
        """Return a list of path points (x, y) in world frame.

        Requirements:
        - Path length: 5 to 10 meters (default 7.0m).
        - Step size: <= 0.5 meters (default 0.25m).
        - Handles 0, 1, 2, 3+ cones on either or both sides.
        - Starts at car position and aligns with car heading.
        """
        cx, cy, cyaw = self.car_pose.x, self.car_pose.y, self.car_pose.yaw
        cos_a, sin_a = math.cos(cyaw), math.sin(cyaw)

        # Coordinate transformation helper functions
        def world_to_local(wx: float, wy: float) -> Tuple[float, float]:
            dx, dy = wx - cx, wy - cy
            lx = dx * cos_a + dy * sin_a
            ly = -dx * sin_a + dy * cos_a
            return lx, ly

        def local_to_world(lx: float, ly: float) -> Tuple[float, float]:
            wx = cx + lx * cos_a - ly * sin_a
            wy = cy + lx * sin_a + ly * cos_a
            return wx, wy

        # Separate cones into local blue (left) and yellow (right) boundaries
        blue_local: List[Tuple[float, float]] = []
        yellow_local: List[Tuple[float, float]] = []

        for cone in self.cones:
            lx, ly = world_to_local(cone.x, cone.y)
            if cone.color == 1:
                blue_local.append((lx, ly))
            else:
                yellow_local.append((lx, ly))

        # Sort cones along forward progress lx
        blue_local.sort(key=lambda p: p[0])
        yellow_local.sort(key=lambda p: p[0])

        # Estimate track width from overlapping blue and yellow cone pairs
        widths = []
        for bx, by in blue_local:
            for yx, yy in yellow_local:
                if abs(bx - yx) < 2.5:
                    w = by - yy
                    if w > 0.5:
                        widths.append(w)

        if widths and _HAS_SCIPY:
            track_width = float(np.median(widths))
        elif widths:
            widths.sort()
            track_width = widths[len(widths) // 2]
        else:
            track_width = 2.0  # Default assumed track width in meters

        track_width = max(1.0, min(3.5, track_width))
        half_width = track_width / 2.0

        target_length = 7.0
        step_size = 0.25

        # Initialize waypoints in car local frame starting at (0, 0)
        waypoints: List[Tuple[float, float]] = [(0.0, 0.0)]

        if not blue_local and not yellow_local:
            # Scenario: No cones visible -> move straight ahead
            waypoints.append((target_length, 0.0))

        elif blue_local and yellow_local:
            # Both boundaries present: match pairs or use normal offsets for unpaired cones
            used_yellow = set()
            for bx, by in blue_local:
                best_j = None
                best_dist = 2.5
                for j, (yx, yy) in enumerate(yellow_local):
                    if j in used_yellow:
                        continue
                    dist = abs(bx - yx)
                    if dist < best_dist:
                        best_dist = dist
                        best_j = j

                if best_j is not None:
                    used_yellow.add(best_j)
                    yx, yy = yellow_local[best_j]
                    waypoints.append(((bx + yx) / 2.0, (by + yy) / 2.0))
                else:
                    # Unmatched blue cone: shift right (towards yellow side) by half_width
                    waypoints.append((bx, by - half_width))

            for j, (yx, yy) in enumerate(yellow_local):
                if j not in used_yellow:
                    # Unmatched yellow cone: shift left (towards blue side) by half_width
                    waypoints.append((yx, yy + half_width))

        elif blue_local:
            # Only blue cones (left boundary given)
            # Offset left boundary inward (to the right) along local normals
            normals = self._compute_right_normals([(0.0, 0.0)] + blue_local)[1:]
            for (bx, by), (nx, ny) in zip(blue_local, normals):
                waypoints.append((bx + nx * half_width, by + ny * half_width))

        elif yellow_local:
            # Only yellow cones (right boundary given)
            # Offset right boundary inward (to the left) along local normals
            normals = self._compute_right_normals([(0.0, 0.0)] + yellow_local)[1:]
            for (yx, yy), (nx, ny) in zip(yellow_local, normals):
                waypoints.append((yx - nx * half_width, yy - ny * half_width))

        # Filter out points behind the car (lx < -0.5) except for start at (0, 0)
        valid_wp: List[Tuple[float, float]] = [(0.0, 0.0)]
        for wx, wy in waypoints:
            if (wx, wy) == (0.0, 0.0):
                continue
            if wx >= -0.5:
                valid_wp.append((wx, wy))

        # Sort remaining waypoints by lx, keeping (0, 0) as first element
        body_wp = valid_wp[1:]
        body_wp.sort(key=lambda p: p[0])

        final_wp: List[Tuple[float, float]] = [(0.0, 0.0)]
        for pt in body_wp:
            if pt[0] <= 0.01:
                continue
            if pt[0] - final_wp[-1][0] > 0.05 or abs(pt[1] - final_wp[-1][1]) > 0.05:
                final_wp.append(pt)

        # Extend waypoints to reach target_length
        if len(final_wp) == 1:
            final_wp.append((target_length, 0.0))
        else:
            last_x, last_y = final_wp[-1]
            if last_x < target_length:
                prev_x, prev_y = final_wp[-2]
                dx, dy = last_x - prev_x, last_y - prev_y
                dist = math.hypot(dx, dy)
                ux, uy = (dx / dist, dy / dist) if dist > 1e-3 else (1.0, 0.0)
                rem = target_length - last_x
                final_wp.append((last_x + ux * rem, last_y + uy * rem))

        # Interpolate waypoints smoothly
        wx_list = [p[0] for p in final_wp]
        wy_list = [p[1] for p in final_wp]

        s_list = [0.0]
        for i in range(1, len(wx_list)):
            ds = math.hypot(wx_list[i] - wx_list[i - 1], wy_list[i] - wy_list[i - 1])
            s_list.append(s_list[-1] + max(ds, 1e-3))

        total_s = target_length
        num_samples = int(math.ceil(total_s / step_size)) + 1
        num_samples = max(num_samples, 20)

        if _HAS_SCIPY:
            s_samples = np.linspace(0.0, total_s, num_samples)
        else:
            s_samples = [i * (total_s / (num_samples - 1)) for i in range(num_samples)]

        if len(wx_list) >= 3 and _HAS_SCIPY:
            try:
                # Clamped boundary condition at s=0 matching car heading (dx/ds=1, dy/ds=0)
                cs_x = CubicSpline(s_list, wx_list, bc_type=((1, 1.0), (2, 0.0)))
                cs_y = CubicSpline(s_list, wy_list, bc_type=((1, 0.0), (2, 0.0)))
                lx_pts = cs_x(s_samples)
                ly_pts = cs_y(s_samples)
            except Exception:
                lx_pts = np.interp(s_samples, s_list, wx_list)
                ly_pts = np.interp(s_samples, s_list, wy_list)
        else:
            if _HAS_SCIPY:
                lx_pts = np.interp(s_samples, s_list, wx_list)
                ly_pts = np.interp(s_samples, s_list, wy_list)
            else:
                lx_pts = [self._linear_interp(s, s_list, wx_list) for s in s_samples]
                ly_pts = [self._linear_interp(s, s_list, wy_list) for s in s_samples]

        # Enforce exact local start position
        lx_pts[0] = 0.0
        ly_pts[0] = 0.0

        # Transform path points back to world frame
        path: Path2D = []
        for lx, ly in zip(lx_pts, ly_pts):
            wx_world, wy_world = local_to_world(lx, ly)
            path.append((wx_world, wy_world))

        return path

    def _compute_right_normals(self, pts: List[Tuple[float, float]]) -> List[Tuple[float, float]]:
        """Compute unit right normal vectors for a sequence of 2D points."""
        n = len(pts)
        if n == 0:
            return []
        if n == 1:
            return [(0.0, -1.0)]

        normals = []
        for i in range(n):
            if i == 0:
                dx = pts[1][0] - pts[0][0]
                dy = pts[1][1] - pts[0][1]
            elif i == n - 1:
                dx = pts[-1][0] - pts[-2][0]
                dy = pts[-1][1] - pts[-2][1]
            else:
                dx = pts[i + 1][0] - pts[i - 1][0]
                dy = pts[i + 1][1] - pts[i - 1][1]

            length = math.hypot(dx, dy)
            if length < 1e-6:
                normals.append((0.0, -1.0))
            else:
                # Unit tangent (tx, ty); Unit right normal is (ty, -tx)
                tx, ty = dx / length, dy / length
                normals.append((ty, -tx))
        return normals

    @staticmethod
    def _linear_interp(x: float, xp: List[float], fp: List[float]) -> float:
        """Fallback 1D linear interpolation if numpy is unavailable."""
        if x <= xp[0]:
            return fp[0]
        if x >= xp[-1]:
            return fp[-1]
        for i in range(len(xp) - 1):
            if xp[i] <= x <= xp[i + 1]:
                t = (x - xp[i]) / (xp[i + 1] - xp[i])
                return fp[i] + t * (fp[i + 1] - fp[i])
        return fp[-1]
