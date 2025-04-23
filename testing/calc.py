import sys
import numpy as np
from PyQt5.QtWidgets import QApplication, QMainWindow, QWidget, QVBoxLayout, QTextEdit, QPushButton, QHBoxLayout
from PyQt5.QtCore import QRectF
import pyqtgraph as pg
from PyQt5.QtWidgets import QGraphicsEllipseItem


class CNCViewer(QMainWindow):
    def __init__(self):
        super().__init__()
        self.setWindowTitle("CNC Profile Editor")
        self.resize(1000, 700)

        central_widget = QWidget()
        layout = QVBoxLayout()
        central_widget.setLayout(layout)
        self.setCentralWidget(central_widget)

        # Textbox para editar o G-code
        self.text_edit = QTextEdit()
        self.text_edit.setPlainText(
            "G0 X0 Z0\n"
            "G1 X14 Z0\n"
            "G1 X14 Z-35.55\n"
            "G2 X10 Z-40.818 R10\n"
            "G1 X10 Z-71.05\n"
            "G2 X14 Z-77.05 R10\n"
            "G1 X14 Z-90\n"
            "G1 X0 Z-90"
        )
        layout.addWidget(self.text_edit)

        # Botão para atualizar o desenho
        button_layout = QHBoxLayout()
        update_button = QPushButton("Atualizar Desenho")
        update_button.clicked.connect(self.update_plot)
        button_layout.addWidget(update_button)
        layout.addLayout(button_layout)

        # Widget de plot
        self.plot_widget = pg.GraphicsLayoutWidget()
        self.plot = self.plot_widget.addPlot()
        self.plot.setLabel('left', 'X')
        self.plot.setLabel('bottom', 'Z')
        self.plot.showGrid(x=True, y=True)
        self.plot.setAspectLocked(True)
        layout.addWidget(self.plot_widget)

        # Desenha o gráfico pela primeira vez
        self.update_plot()

    def parse_gcode(self, text):
        toolpath = []
        for line in text.strip().splitlines():
            tokens = line.strip().split()
            if not tokens:
                continue
            move = {'cmd': tokens[0]}
            for t in tokens[1:]:
                if t[0] in 'XZRF':
                    move[t[0]] = float(t[1:])
            toolpath.append(move)
        return toolpath

    def draw_arc(self, x0, z0, x1, z1, r, clockwise):
        # Vector between start and end
        dx = x1 - x0
        dz = z1 - z0
        chord_length = np.hypot(dx, dz)
        if chord_length == 0:
            print("Invalid arc (zero-length).")
            return

        # Midpoint
        mx = (x0 + x1) / 2
        mz = (z0 + z1) / 2

        # Height from chord to arc center
        try:
            h = np.sqrt(r**2 - (chord_length / 2)**2)
        except ValueError:
            print("Arc radius too small for given endpoints.")
            return

        # Unit perpendicular vector (normal to the chord)
        nx = -dz / chord_length
        nz = dx / chord_length
        if clockwise:
            nx, nz = -nx, -nz

        # Arc center
        cx = mx + h * nx
        cz = mz + h * nz

        # Angles (note: Z horizontal, X vertical)
        theta0 = np.arctan2(z0 - cz, x0 - cx)
        theta1 = np.arctan2(z1 - cz, x1 - cx)

        # Fix angle direction
        if clockwise:
            while theta1 > theta0:
                theta1 -= 2 * np.pi
        else:
            while theta1 < theta0:
                theta1 += 2 * np.pi

        # Generate arc
        theta = np.linspace(theta0, theta1, 100)
        arc_z = cz + r * np.cos(theta)
        arc_x = cx + r * np.sin(theta)
        self.plot.plot(arc_z, arc_x, pen=pg.mkPen('r', width=2))

        # Mark center
        self.plot.plot([cz], [cx], pen=None, symbol='o', symbolBrush='g')

        # Mark start/end
        self.plot.plot([z0], [x0], pen=None, symbol='o', symbolBrush='b')
        self.plot.plot([z1], [x1], pen=None, symbol='x', symbolBrush='r')



    def update_plot(self):
        self.plot.clear()
        gcode_text = self.text_edit.toPlainText()
        toolpath = self.parse_gcode(gcode_text)
        last_point = None

        for move in toolpath:
            x, z = move.get('X'), move.get('Z')
            if move['cmd'] == 'G0':
                last_point = (x, z)
                continue
            if move['cmd'] == 'G1' and last_point:
                self.plot.plot([last_point[1], z], [last_point[0], x], pen=pg.mkPen('y', width=2))
            elif move['cmd'] == 'G2' and last_point:
                self.draw_arc(last_point[0], last_point[1], x, z, move['R'], clockwise=True)
            elif move['cmd'] == 'G3' and last_point:
                self.draw_arc(last_point[0], last_point[1], x, z, move['R'], clockwise=False)


            last_point = (x, z)


if __name__ == '__main__':
    app = QApplication(sys.argv)
    viewer = CNCViewer()
    viewer.show()
    sys.exit(app.exec_())
