# lead_generator.py

import math
import numpy as np


def compute_lead_point(mode, ref_point, direction_vec, distance, angle_deg=0.0, incremental=True):
    """
    Returns a new point offset from ref_point along a lead-in or lead-out path.

    Parameters:
        mode: 'Direct', 'Angle', 'Perpendicular', 'Tangential', 'Auto'
        ref_point: tuple (x, z) where the segment starts or ends
        direction_vec: unit vector (dx, dz) of segment direction
        distance: lead length
        angle_deg: user angle in degrees (only for 'Angle' mode)
        incremental: whether distance is relative to ref_point (True) or absolute (False)
    """
    """
    Computes a lead-in/out point based on strategy.
    Ensures direction is away from profile for entry, toward outside.
    """
    import math
    dx, dz = direction_vec
    x0, z0 = ref_point

    if mode == "Auto":
        # fallback to Tangential
        mode = "Tangential"

    if mode == "Direct":
        offset = (0, -distance)  # Z up, assuming profile goes downward

    elif mode == "Angle":
        angle_rad = math.radians(angle_deg)
        # Rotate direction vector *backward*
        offset_dx = math.cos(angle_rad) * dx - math.sin(angle_rad) * dz
        offset_dz = math.sin(angle_rad) * dx + math.cos(angle_rad) * dz
        offset = (-offset_dx * distance, -offset_dz * distance)

    elif mode == "Perpendicular":
        # Perpendicular vector to segment, pointing outward
        perp_dx, perp_dz = dz, -dx
        offset = (-perp_dx * distance, -perp_dz * distance)

    elif mode == "Tangential":
        offset = (-dx * distance, -dz * distance)

    else:
        offset = (0, 0)

    if incremental:
        return x0 + offset[0], z0 + offset[1]
    else:
        return offset[0], offset[1]


def compute_unit_vector(p1, p2):
    """Return unit direction vector from p1 to p2"""
    dx = p2[0] - p1[0]
    dz = p2[1] - p1[1]
    length = np.hypot(dx, dz)
    return (dx / length, dz / length) if length > 1e-6 else (0, 1)

