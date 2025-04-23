
import numpy as np
import json
import math
import os

def load_material_names(json_path='materials.json'):
    json_path = os.path.join(os.path.dirname(__file__), json_path)
    with open(json_path, 'r') as f:
        data = json.load(f)
    return list(data.keys())

def load_material_data(name, material_file='materials.json'):

    json_path = os.path.join(os.path.dirname(__file__), material_file)

    with open(json_path, 'r') as file:
        materials = json.load(file)
    return materials.get(name)


def calculate_spindle_speed(cutting_speed_m_per_min, diameter_mm):
    return (1000 * cutting_speed_m_per_min) / (math.pi * diameter_mm)

def calculate_feed_rate(spindle_speed_rpm, feed_per_rev_mm):
    return spindle_speed_rpm * feed_per_rev_mm

def order_segments_monotonic_z(segments):
    segments = segments.copy()
    segments.sort(key=lambda seg: seg[1][1])  # sort by Z start
    ordered = []

    start_idx = min(range(len(segments)), key=lambda i: abs(segments[i][1][1]))
    start_seg = segments.pop(start_idx)

    # Flip if Z is increasing
    if start_seg[2][1] < start_seg[1][1]:
        ordered.append(start_seg)
        current_point = start_seg[2]
    else:
        if start_seg[0] == 'LINE':
            start_seg = ('LINE', start_seg[2], start_seg[1])
        elif start_seg[0] == 'ARC':
            start_seg = ('ARC', start_seg[2], start_seg[1], start_seg[3])
        ordered.append(start_seg)
        current_point = start_seg[2]

    while segments:
        best_idx = -1
        best_dist = float('inf')
        reverse = False

        for i, seg in enumerate(segments):
            d1 = np.linalg.norm(np.array(current_point) - np.array(seg[1]))
            d2 = np.linalg.norm(np.array(current_point) - np.array(seg[2]))
            if d1 < best_dist:
                best_idx, best_dist, reverse = i, d1, False
            if d2 < best_dist:
                best_idx, best_dist, reverse = i, d2, True

        if best_idx == -1:
            break

        seg = segments.pop(best_idx)
        if reverse:
            if seg[0] == 'LINE':
                seg = ('LINE', seg[2], seg[1])
            elif seg[0] == 'ARC':
                seg = ('ARC', seg[2], seg[1], seg[3])

        # Enforce Z descent
        if seg[2][1] <= current_point[1] + 1e-4:
            ordered.append(seg)
            current_point = seg[2]

    return ordered

def generate_gcode_from_segments(segments, material_profile, tool_diameter_mm, scale=2.0):
    if not segments:
        return []

    gcode = []
    

    # === Calculate Speeds ===
    rpm = calculate_spindle_speed(material_profile["surface_speed_m_per_min"], tool_diameter_mm)
    feed_rate = calculate_feed_rate(rpm, material_profile["feed_per_rev_mm"])

    # === Order Segments ===
    ordered = order_segments_monotonic_z(segments)
    start_x, start_z = ordered[0][1]

    # === Header and Spindle Start ===
    gcode.append(f"G0 X{scale * start_x:.3f} Z{start_z:.3f}")
    gcode.append(f"S{rpm:.0f} M3   ; Spindle on clockwise")
    gcode.append(f"G1 Z{ordered[0][2][1]:.3f} F{feed_rate:.1f}")

    last_point = (start_x, ordered[0][2][1])

    # === Profile ===
    for seg in ordered:
        if seg[0] == "LINE":
            _, _, p2 = seg
            if p2 != last_point:
                gcode.append(f"G1 X{scale * p2[0]:.3f} Z{p2[1]:.3f}")
                last_point = p2

        elif seg[0] == "ARC":
            _, p1, p2, r = seg
            clockwise = p2[1] < p1[1]
            cmd = "G2" if clockwise else "G3"
            gcode.append(f"{cmd} X{scale * p2[0]:.3f} Z{p2[1]:.3f} R{r:.3f}")
            last_point = p2

    # === Final Retract and End ===
    gcode.append(f"G0 Z5.000   ; Safe retract")
    gcode.append("M5          ; Spindle stop")
    gcode.append("M30         ; End of program")

    return gcode
