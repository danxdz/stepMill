import sys
import numpy as np
from PyQt5.QtWidgets import (
    QApplication, QMainWindow, QWidget, QVBoxLayout, QTextEdit,
    QPushButton, QHBoxLayout, QFileDialog, QDockWidget, QFormLayout, QLineEdit, QCheckBox
)
from PyQt5.QtCore import QTimer
import pyqtgraph as pg

from OCC.Display.backend import load_backend
load_backend("pyqt5")
from OCC.Core.STEPControl import STEPControl_Reader
from OCC.Core.IFSelect import IFSelect_RetDone
from OCC.Core.BRepAlgoAPI import BRepAlgoAPI_Section
from OCC.Core.gp import gp_Pln, gp_Pnt, gp_Dir
from OCC.Core.TopExp import TopExp_Explorer
from OCC.Core.TopAbs import TopAbs_EDGE
from OCC.Core.BRepAdaptor import BRepAdaptor_Curve
from OCC.Core.GeomAbs import GeomAbs_Line, GeomAbs_Circle

from lead_generator import compute_lead_point, compute_unit_vector
from gcode_toolpath_block import extract_toolpath_block, replace_toolpath_block


class CNCViewer(QMainWindow):
    def __init__(self):
        super().__init__()
        self.setWindowTitle("CNC Profile from STEP")
        self.resize(1200, 850)
        self.last_shape = None

        central = QWidget()
        layout = QVBoxLayout(central)
        self.setCentralWidget(central)

        self.text_edit = QTextEdit()
        self.text_edit.cursorPositionChanged.connect(self.highlight_selected_point)
        self.text_edit.textChanged.connect(self.update_plot)
        layout.addWidget(self.text_edit)

        btn_layout = QHBoxLayout()
        self.load_btn = QPushButton("Load STEP File")
        self.load_btn.clicked.connect(self.load_step_file)
        self.play_button = QPushButton("\u25b6 Animate Tool")
        self.play_button.clicked.connect(self.play_animation)
        btn_layout.addWidget(self.load_btn)
        btn_layout.addWidget(self.play_button)
        layout.addLayout(btn_layout)

        self.reset_button = QPushButton("Reset G-code")
        self.reset_button.clicked.connect(self.reset_gcode)
        btn_layout.addWidget(self.reset_button)


        self.plot_widget = pg.GraphicsLayoutWidget()
        self.plot = self.plot_widget.addPlot()
        self.plot.setLabel('left', 'X (Diameter)')
        self.plot.setLabel('bottom', 'Z')
        self.plot.showGrid(x=True, y=True)
        self.plot.setAspectLocked(True)
        self.plot.addLine(x=0, pen=pg.mkPen('w', style=pg.QtCore.Qt.DotLine))
        layout.addWidget(self.plot_widget)

        self.init_param_panel()

    def init_param_panel(self):
        from PyQt5.QtWidgets import (
            QComboBox, QGroupBox, QFormLayout, QVBoxLayout, QHBoxLayout, QLabel
        )
        from PyQt5.QtCore import Qt

        self.dock = QDockWidget("Toolpath Parameters", self)
        self.param_widget = QWidget()
        dock_layout = QVBoxLayout(self.param_widget)
        dock_layout.setContentsMargins(10, 10, 10, 10)
        dock_layout.setSpacing(12)

        # =============== Enable and Sync Checkboxes ===============
        self.enable_leads = QCheckBox("Enable Lead-In/Out")
        self.link_leads = QCheckBox("Link In/Out Settings")
        self.link_leads.setChecked(True)
        dock_layout.addWidget(self.enable_leads)
        dock_layout.addWidget(self.link_leads)

        # =============== Lead-In Group ===============
        in_group = QGroupBox("Lead-In Settings")
        in_group.setStyleSheet("QGroupBox { font-weight: bold; margin-top: 12px; }")
        in_form = QFormLayout()

        self.lead_in_mode = QComboBox()
        self.lead_in_mode.addItems(["Auto", "Direct", "Angle", "Perpendicular", "Tangential"])
        self.lead_in_dist = QLineEdit("5.0")
        self.lead_in_angle = QLineEdit("45.0")
        in_form.addRow("Mode:", self.lead_in_mode)
        in_form.addRow("Distance:", self.lead_in_dist)
        self.lead_in_angle_label = QLabel("Angle (°):")
        in_form.addRow(self.lead_in_angle_label, self.lead_in_angle)
        in_group.setLayout(in_form)
        dock_layout.addWidget(in_group)

        # =============== Lead-Out Group (Hidden by Default) ===============
        out_group = QGroupBox("Lead-Out Settings")
        out_group.setStyleSheet("QGroupBox { font-weight: bold; margin-top: 12px; }")
        out_form = QFormLayout()

        self.lead_out_mode = QComboBox()
        self.lead_out_mode.addItems(["Auto", "Direct", "Angle", "Perpendicular", "Tangential"])
        self.lead_out_dist = QLineEdit("3.0")
        self.lead_out_angle = QLineEdit("90.0")
        out_form.addRow("Mode:", self.lead_out_mode)
        out_form.addRow("Distance:", self.lead_out_dist)
        self.lead_out_angle_label = QLabel("Angle (°):")
        out_form.addRow(self.lead_out_angle_label, self.lead_out_angle)
        out_group.setLayout(out_form)
        dock_layout.addWidget(out_group)
        out_group.setVisible(False)


        self.incremental_leads = QCheckBox("Use Incremental Leads")
        self.incremental_leads.setChecked(True)



        # =============== General Settings ===============
        general_group = QGroupBox("General Settings")
        general_group.setStyleSheet("QGroupBox { font-weight: bold; margin-top: 12px; }")
        general_form = QFormLayout()

        self.lead_radius = QLineEdit("0.0")
        self.safe_retract_x = QLineEdit("18.0")
        self.feed_in = QLineEdit("200")
        self.feed_out = QLineEdit("300")
        self.final_retract_z = QLineEdit("5.0")

        general_form.addRow("Safe Retract X:", self.safe_retract_x)
        general_form.addRow("Safe Retract Z:", self.final_retract_z)
        general_form.addRow("Feed-In Rate:", self.feed_in)
        general_form.addRow("Feed-Out Rate:", self.feed_out)
        general_group.setLayout(general_form)
        dock_layout.addWidget(general_group)

        dock_layout.addStretch(1)
        self.dock.setWidget(self.param_widget)
        self.addDockWidget(Qt.RightDockWidgetArea, self.dock)

        general_form.addRow(self.incremental_leads)


        # =============== Sync and Toggle Behavior ===============
        def sync_out_params():
            linked = self.link_leads.isChecked()
            out_group.setVisible(not linked)
            if linked:
                self.lead_out_mode.setCurrentIndex(self.lead_in_mode.currentIndex())
                self.lead_out_dist.setText(self.lead_in_dist.text())
                self.lead_out_angle.setText(self.lead_in_angle.text())

        def update_angle_visibility():
            in_mode = self.lead_in_mode.currentText()
            out_mode = self.lead_out_mode.currentText()

            # Show angle field only in "Angle" mode
            self.lead_in_angle.setVisible(in_mode == "Angle")
            self.lead_in_angle_label.setVisible(in_mode == "Angle")

            self.lead_out_angle.setVisible(out_mode == "Angle")
            self.lead_out_angle_label.setVisible(out_mode == "Angle")

        self.link_leads.stateChanged.connect(sync_out_params)
        self.lead_in_mode.currentIndexChanged.connect(sync_out_params)
        self.lead_in_mode.currentIndexChanged.connect(update_angle_visibility)
        self.lead_out_mode.currentIndexChanged.connect(update_angle_visibility)

        # =============== Regenerate on Any Change ===============
        controls = [
            self.enable_leads, self.link_leads,
            self.lead_in_mode, self.lead_out_mode,
            self.lead_in_dist, self.lead_out_dist,
            self.lead_in_angle, self.lead_out_angle,
            self.lead_radius, self.safe_retract_x,
            self.feed_in, self.feed_out, self.final_retract_z
        ]
        
        controls.append(self.incremental_leads)
        for ctrl in controls:
            if isinstance(ctrl, QLineEdit):
                ctrl.textChanged.connect(self.generate_gcode)
            elif isinstance(ctrl, QCheckBox):
                ctrl.stateChanged.connect(self.generate_gcode)
            elif isinstance(ctrl, QComboBox):
                ctrl.currentIndexChanged.connect(self.generate_gcode)

        # Initial angle visibility setup
        update_angle_visibility()

    def reset_gcode(self):
        if hasattr(self, 'initial_gcode'):
            self.text_edit.setPlainText(self.initial_gcode)
            self.update_plot()


    def load_step_file(self):
        path, _ = QFileDialog.getOpenFileName(self, "Open STEP File", "", "STEP Files (*.step *.stp)")
        if not path:
            return

        reader = STEPControl_Reader()
        if reader.ReadFile(path) != IFSelect_RetDone:
            return
        reader.TransferRoots()
        self.last_shape = reader.OneShape()

        self.extract_profile()
        self.generate_gcode()
        self.initial_gcode = self.text_edit.toPlainText()


    def extract_profile(self):
        self.profile_segments = []
        plane = gp_Pln(gp_Pnt(0, 0, 0), gp_Dir(0, 1, 0))
        section = BRepAlgoAPI_Section(self.last_shape, plane, False)
        section.ComputePCurveOn1(True)
        section.Approximation(True)
        section.Build()

        explorer = TopExp_Explorer(section.Shape(), TopAbs_EDGE)
        while explorer.More():
            edge = explorer.Current()
            adaptor = BRepAdaptor_Curve(edge)
            ctype = adaptor.GetType()
            f, l = adaptor.FirstParameter(), adaptor.LastParameter()
            p1, p2 = adaptor.Value(f), adaptor.Value(l)
            x1, z1 = abs(p1.X()), p1.Z()
            x2, z2 = abs(p2.X()), p2.Z()

            if x1 < 1e-4 and x2 < 1e-4:
                explorer.Next()
                continue

            if ctype == GeomAbs_Line:
                self.profile_segments.append(('LINE', (x1, z1), (x2, z2)))
            elif ctype == GeomAbs_Circle:
                try:
                    radius = adaptor.Circle().Radius()
                    self.profile_segments.append(('ARC', (x1, z1), (x2, z2), radius))
                except:
                    pass
            explorer.Next()

    def generate_gcode(self):
        if not hasattr(self, 'profile_segments') or not self.profile_segments:
            return

        # === Parameters ===
        try:
            use_lead = self.enable_leads.isChecked()
            lead_in_mode = self.lead_in_mode.currentText()
            lead_out_mode = self.lead_out_mode.currentText()
            lead_in_dist = float(self.lead_in_dist.text())
            lead_out_dist = float(self.lead_out_dist.text())
            lead_in_angle = float(self.lead_in_angle.text()) if lead_in_mode == "Angle" else 0.0
            lead_out_angle = float(self.lead_out_angle.text()) if lead_out_mode == "Angle" else 0.0
            safe_x = float(self.safe_retract_x.text())  # final X
            retract_z = float(self.final_retract_z.text())  # final Z
            feed_in = float(self.feed_in.text())
            feed_out = float(self.feed_out.text())
            incremental = self.incremental_leads.isChecked()
        except ValueError:
            return

        segments = self.profile_segments.copy()
        if not segments:
            return

        # === Reorder profile from closest Z=0 ===
        start_index = min(range(len(segments)), key=lambda i: abs(segments[i][1][1]))
        ordered = [segments.pop(start_index)]
        current_point = ordered[0][2]

        while segments:
            best_idx, best_dist, reverse = -1, float('inf'), False
            for i, seg in enumerate(segments):
                d_start = np.linalg.norm(np.array(current_point) - np.array(seg[1]))
                d_end = np.linalg.norm(np.array(current_point) - np.array(seg[2]))
                if d_start < best_dist:
                    best_idx, best_dist, reverse = i, d_start, False
                if d_end < best_dist:
                    best_idx, best_dist, reverse = i, d_end, True
            if best_idx == -1:
                break
            seg = segments.pop(best_idx)
            if reverse:
                if seg[0] == 'LINE':
                    seg = ('LINE', seg[2], seg[1])
                elif seg[0] == 'ARC':
                    seg = ('ARC', seg[2], seg[1], seg[3])
            ordered.append(seg)
            current_point = seg[2]

        ordered = [seg for seg in ordered if seg[1] != seg[2]]
        if not ordered:
            return

        gcode = []
        first_seg = ordered[0]
        first_x, first_z = first_seg[1]
        second_x, second_z = first_seg[2]
        last_point = None

       # === LEAD-IN ===
        if use_lead:
            dir_vec = compute_unit_vector((first_x, first_z), (second_x, second_z))
            if self.link_leads.isChecked():
                lead_out_mode = lead_in_mode
                lead_out_dist = lead_in_dist
                lead_out_angle = lead_in_angle

            lead_start_x, lead_start_z = compute_lead_point(
                lead_in_mode, (first_x, first_z), dir_vec,
                lead_in_dist, lead_in_angle, incremental
            )

            # Move to tool-change/home position
            gcode.append(f"G0 X{safe_x:.3f} Z{retract_z:.3f}   ; Move to safe retract start position")
            # Rapid move above lead start
            gcode.append(f"G0 X{2 * lead_start_x:.3f} Z{retract_z:.3f}")
            # Feed down to lead start
            if lead_start_z != retract_z:
                gcode.append(f"G1 X{2 * lead_start_x:.3f} Z{lead_start_z + 15.0:.3f}")
                gcode.append(f"G1 X{2 * lead_start_x:.3f} Z{lead_start_z:.3f} F{feed_in:.1f}")
            # Move to profile start
            gcode.append(f"G1 X{2 * first_x:.3f} Z{first_z:.3f}")
            skip_first = True
        else:
            # Move in X only, Z stays at surface
            gcode.append(f"G0 X{2 * first_x + 5.0:.3f} Z{first_z:.3f}")
            gcode.append(f"G1 X{2 * first_x:.3f} Z{first_z:.3f} F{feed_in:.1f}")





        # === PROFILE ===
        skip_first = use_lead
        for idx, seg in enumerate(ordered):
            if skip_first and idx == 0 and seg[1] == seg[2]:
                last_point = seg[2]
                continue
                    
            if seg[0] == 'LINE':
                _, _, (x, z) = seg
                if not last_point or (x, z) != last_point:
                    gcode.append(f"G1 X{2 * x:.3f} Z{z:.3f}")
                    last_point = (x, z)

            elif seg[0] == 'ARC':
                _, start, end, r = seg
                clockwise = end[1] < start[1]
                gcode.append(f"{'G2' if clockwise else 'G3'} X{2 * end[0]:.3f} Z{end[1]:.3f} R{r:.3f}")
                last_point = end

        # === LEAD-OUT ===
        if use_lead and last_point:
            last_x, last_z = last_point
            dir_vec = compute_unit_vector(ordered[-1][1], ordered[-1][2]) if ordered else (1.0, 0.0)

            if lead_out_mode == "Auto":
                if abs(dir_vec[0]) < 1e-3:
                    lead_out_x, lead_out_z = last_x + 2.0, last_z
                else:
                    lead_out_x, lead_out_z = compute_lead_point(
                        "Tangential", (last_x, last_z), dir_vec,
                        lead_out_dist, 0.0, incremental
                    )
            else:
                lead_out_x, lead_out_z = compute_lead_point(
                    lead_out_mode, (last_x, last_z), dir_vec,
                    lead_out_dist, lead_out_angle, incremental
                )

            gcode.append(f"G1 X{2 * lead_out_x:.3f} Z{lead_out_z:.3f} F{feed_out:.1f}   ; lead-out move")

            # Step 1: Retract radially in X (stay at current Z)
            if abs(2 * lead_out_x - safe_x) > 0.1:
                gcode.append(f"G0 X{safe_x:.3f} Z{lead_out_z:.3f}   ; radial retract")

            # Step 2: Retract axially in Z
            if abs(lead_out_z - retract_z) > 0.1:
                gcode.append(f"G0 X{safe_x:.3f} Z{retract_z:.3f}   ; axial retract")




        # === Final Retract ===
        existing_code = self.text_edit.toPlainText()
        updated_code = replace_toolpath_block(existing_code, gcode)
        self.text_edit.setPlainText(updated_code)

        self.update_plot()



    def parse_gcode_to_segments(self):
        lines = self.text_edit.toPlainText().splitlines()
        segments = []
        last_point = None

        for line in lines:
            line = line.strip()
            if not line or not any(cmd in line for cmd in ("G0", "G1", "G2", "G3")):
                continue

            tokens = line.split()
            x = z = r = None
            cmd = None

            for tok in tokens:
                if tok.startswith("G"):
                    cmd = tok
                elif tok.startswith("X"):
                    x = float(tok[1:]) / 2
                elif tok.startswith("Z"):
                    z = float(tok[1:])
                elif tok.startswith("R"):
                    r = float(tok[1:])

            if x is not None and z is not None:
                current_point = (x, z)
                if last_point is not None:
                    if cmd == "G1":
                        segments.append(("LINE", last_point, current_point))
                    elif cmd == "G0":
                        segments.append(("RAPID", last_point, current_point))
                    elif cmd in ("G2", "G3") and r is not None:
                        clockwise = cmd == "G2"
                        segments.append(("ARC", last_point, current_point, r, clockwise))
                last_point = current_point
            else:
                last_point = None

        return segments

    def update_plot(self):
        self.plot.clear()
        self.plot.addLine(x=0, pen=pg.mkPen('w', style=pg.QtCore.Qt.DotLine))

        segments = self.parse_gcode_to_segments()
        for seg in segments:
            if seg[0] == 'LINE':
                _, p1, p2 = seg
                pen = 'y'
                if 'lead_in' in seg:
                    pen = 'g'
                elif 'lead_out' in seg:
                    pen = 'b'
                self.plot.plot([p1[1], p2[1]], [2 * p1[0], 2 * p2[0]], pen=pen)

            elif seg[0] == 'ARC':
                _, p1, p2, r, clockwise = seg
                arc = self._interpolate_arc(p1, p2, r, clockwise)
                if arc:
                    zz, xx = zip(*arc)
                    pen = 'y'
                    if 'lead_out' in seg:
                        pen = 'b'
                    elif 'lead_in' in seg:
                        pen = 'g'
                    self.plot.plot(zz, [2 * x for x in xx], pen=pen)

            elif seg[0] == 'RAPID':
                _, p1, p2 = seg
                self.plot.plot([p1[1], p2[1]], [2 * p1[0], 2 * p2[0]], pen=pg.mkPen('r', style=pg.QtCore.Qt.DashLine))





    def _interpolate_arc(self, p0, p1, r, clockwise=True, steps=60):
        p0 = np.array([p0[1], p0[0]])
        p1 = np.array([p1[1], p1[0]])
        chord = p1 - p0
        chord_len = np.linalg.norm(chord)
        if chord_len == 0 or abs(r) < chord_len / 2:
            return [tuple(p0), tuple(p1)]
        mid = (p0 + p1) / 2
        h = np.sqrt(r**2 - (chord_len / 2)**2)
        perp = np.array([-chord[1], chord[0]]) / np.linalg.norm(chord)
        center = mid + (-perp if clockwise else perp) * h
        a0 = np.arctan2(p0[1] - center[1], p0[0] - center[0])
        a1 = np.arctan2(p1[1] - center[1], p1[0] - center[0])
        if clockwise and a1 > a0:
            a1 -= 2 * np.pi
        elif not clockwise and a1 < a0:
            a1 += 2 * np.pi
        theta = np.linspace(a0, a1, steps)
        return [(center[0] + r * np.cos(t), center[1] + r * np.sin(t)) for t in theta]

    def highlight_selected_point(self):
        self.update_plot()
        cursor = self.text_edit.textCursor()
        line = cursor.block().text().strip()
        if not line or not any(cmd in line for cmd in ('G0','G1', 'G2', 'G3')):
            return
        tokens = line.split()
        x = z = None
        for tok in tokens:
            if tok.startswith("X"):
                x = float(tok[1:])
            elif tok.startswith("Z"):
                z = float(tok[1:])
        if x is not None and z is not None:
            self.plot.addItem(pg.ScatterPlotItem([z], [x], size=10, brush='r'))

    def play_animation(self):
        self.update_plot()
        dot = pg.ScatterPlotItem(size=8, brush='r')
        self.plot.addItem(dot)
        points = []

        segments = self.parse_gcode_to_segments()
        total_length = 0
        lengths = []

        # Step 1: Compute segment lengths
        for seg in segments:
            if seg[0] == 'LINE' or seg[0] == 'RAPID':
                _, p1, p2 = seg
                dx = (p2[0] - p1[0]) * 2  # multiply X by 2 to convert from radius to diameter
                dz = p2[1] - p1[1]
                length = np.hypot(dx, dz)
            elif seg[0] == 'ARC':
                _, p1, p2, r, clockwise = seg
                arc = self._interpolate_arc(p1, p2, r, clockwise)
                if not arc:
                    continue
                length = 0
                for i in range(1, len(arc)):
                    dx = (arc[i][0] - arc[i - 1][0]) * 2
                    dz = arc[i][1] - arc[i - 1][1]
                    length += np.hypot(dx, dz)
            else:
                continue

            lengths.append(length)
            total_length += length

        # Step 2: Determine total steps
        total_steps = 500  # Total animation steps (adjust for speed)
        steps_per_mm = total_steps / total_length

        # Step 3: Interpolate points with even spacing
        for i, seg in enumerate(segments):
            if seg[0] == 'LINE' or seg[0] == 'RAPID':
                _, p1, p2 = seg
                dx = (p2[0] - p1[0]) * 2
                dz = p2[1] - p1[1]
                n_steps = max(2, int(lengths[i] * steps_per_mm))
                xs = np.linspace(2 * p1[0], 2 * p2[0], n_steps)
                zs = np.linspace(p1[1], p2[1], n_steps)
                points.extend(zip(zs, xs))
            elif seg[0] == 'ARC':
                _, p1, p2, r, clockwise = seg
                arc = self._interpolate_arc(p1, p2, r, clockwise)
                if not arc:
                    continue
                # Resample arc to desired point count
                arc_len = lengths[i]
                n_steps = max(2, int(arc_len * steps_per_mm))
                arc = np.array(arc)
                zs = arc[:, 0]
                xs = arc[:, 1] * 2
                # Interpolate using cumulative distance
                dist = np.cumsum(np.hstack([[0], np.hypot(np.diff(xs), np.diff(zs))]))
                uniform_d = np.linspace(0, dist[-1], n_steps)
                zs_resampled = np.interp(uniform_d, dist, zs)
                xs_resampled = np.interp(uniform_d, dist, xs)
                points.extend(zip(zs_resampled, xs_resampled))

        # Step 4: Animate
        self._anim_idx = 0
        self._anim_points = points

        def step():
            if self._anim_idx >= len(points):
                timer.stop()
                return
            z, x = points[self._anim_idx]
            dot.setData([z], [x])
            self._anim_idx += 1

        timer = QTimer()
        timer.timeout.connect(step)
        timer.start(20)  # You can reduce this for smoother animation

if __name__ == '__main__':
    app = QApplication(sys.argv)
    viewer = CNCViewer()
    viewer.show()
    sys.exit(app.exec_())
