import sys
from PyQt5.QtWidgets import QApplication, QMainWindow, QPushButton, QTextEdit, QFileDialog, QVBoxLayout, QWidget

from PyQt5.QtWidgets import QLabel, QComboBox, QLineEdit, QHBoxLayout


from stepmill.core.geometry import extract_profile_from_step
from stepmill.core.gcode_generator import generate_gcode_from_segments, load_material_data, load_material_names


class CNCViewer(QMainWindow):
    def __init__(self):
        super().__init__()
        self.setWindowTitle("stepMill")
        self.resize(800, 600)

        self.editor = QTextEdit()
        self.load_btn = QPushButton("Load STEP")
        self.load_btn.clicked.connect(self.load_step)

        layout = QVBoxLayout()
        layout.addWidget(self.load_btn)
        layout.addWidget(self.editor)

        container = QWidget()
        container.setLayout(layout)
        self.setCentralWidget(container)

        self.last_segments = ''


        # Tool settings layout
        tool_layout = QHBoxLayout()

        # Tool diameter input (editable dropdown)
        self.tool_diam_input = QComboBox()
        self.tool_diam_input.setEditable(True)
        self.tool_diam_input.addItems(["20", "16", "12", "10", "8", "6", "5", "4", "3", "2", "1.5", "1.2", "1"])
        self.tool_diam_input.setCurrentText("10")  # default
        tool_layout.addWidget(self.tool_diam_input)

        # Material selector
        self.material_label = QLabel("Material:")
        self.material_select = QComboBox()
        self.material_select.addItems(load_material_names())  # Dynamically load materials
        tool_layout.addWidget(self.material_label)
        tool_layout.addWidget(self.material_select)
        #add listener 
        self.tool_diam_input.currentIndexChanged.connect(self.generate_from_ui)
        self.material_select.currentIndexChanged.connect(self.generate_from_ui)


        layout.addLayout(tool_layout)

    def generate_from_ui(self):
        try:
            tool_diam = float(self.tool_diam_input.currentText())


            material = self.material_select.currentText()
            profile = load_material_data(material)
            segments = self.last_segments  # Assume segments were cached
            gcode = generate_gcode_from_segments(segments, profile, tool_diam)
            self.editor.setPlainText("\n".join(gcode))
        except Exception as e:
            self.editor.setPlainText(f"Error: {e}")


    def load_step(self):
        path, _ = QFileDialog.getOpenFileName(self, "Open STEP File", "", "STEP Files (*.step *.stp)")
        if not path:
            return
        try:
            segments = extract_profile_from_step(path)
            self.last_segments = segments


            material = self.material_select.currentText()
            tool_diam = float(self.tool_diam_input.text())
            profile = load_material_data(material)

            gcode = generate_gcode_from_segments(segments, profile, tool_diam)
            self.editor.setPlainText("\n".join(gcode))
        except Exception as e:
            self.editor.setPlainText(f"Error: {e}")


def main():
    app = QApplication(sys.argv)
    viewer = CNCViewer()
    viewer.show()
    sys.exit(app.exec_())
