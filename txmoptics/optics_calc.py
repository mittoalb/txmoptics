from PyQt5.QtWidgets import (QApplication, QMainWindow, QWidget, QVBoxLayout,
                             QHBoxLayout, QLabel, QLineEdit, QComboBox, QPushButton,
                             QTabWidget, QGridLayout, QGroupBox, QMessageBox,
                             QFileDialog, QDialog, QDialogButtonBox, QTableWidget,
                             QTableWidgetItem, QHeaderView, QFrame, QGraphicsScene,
                             QGraphicsView, QGraphicsTextItem, QGraphicsLineItem,
                             QGraphicsEllipseItem, QGraphicsRectItem, QGraphicsPolygonItem)
from PyQt5.QtCore import Qt, pyqtSignal, QPointF, QRectF
from PyQt5.QtGui import QFont, QPalette, QColor, QPen, QBrush, QPainterPath, QPolygonF, QPainter
import json
import os
import sys
from datetime import datetime


class InstrumentSchematic(QWidget):
    """Dynamic schematic diagram of the X-ray optics instrument"""

    def __init__(self, parent=None):
        super().__init__(parent)
        self.setMinimumHeight(200)
        self.setMaximumHeight(250)

        # Data for display
        self.source_to_condenser = 70  # mm
        self.condenser_to_sample = 0  # mm
        self.sample_to_zp = 0  # mm
        self.zp_to_camera = 3500  # mm
        self.condenser_name = "Sigray"
        self.zp_name = "30nm ZP"
        self.camera_name = "ORX-10G-310S9M"
        self.zp_diameter = 300  # μm
        self.zp_na = 0.1
        self.cond_inner = 450  # μm
        self.cond_outer = 750  # μm

        # Setup graphics
        self.scene = QGraphicsScene()
        self.view = QGraphicsView(self.scene)
        self.view.setRenderHint(QPainter.Antialiasing)
        self.view.setHorizontalScrollBarPolicy(Qt.ScrollBarAlwaysOff)
        self.view.setVerticalScrollBarPolicy(Qt.ScrollBarAlwaysOff)

        layout = QVBoxLayout()
        layout.setContentsMargins(0, 0, 0, 0)
        layout.addWidget(self.view)
        self.setLayout(layout)

        self.draw_schematic()

    def update_parameters(self, source_to_cond, cond_to_sample, sample_to_zp, zp_to_cam,
                         cond_name, zp_name, cam_name, zp_diameter=None, zp_na=None,
                         cond_inner=None, cond_outer=None):
        """Update schematic with new parameters"""
        self.source_to_condenser = source_to_cond
        self.condenser_to_sample = cond_to_sample
        self.sample_to_zp = sample_to_zp
        self.zp_to_camera = zp_to_cam
        self.condenser_name = cond_name
        self.zp_name = zp_name
        self.camera_name = cam_name
        self.zp_diameter = zp_diameter if zp_diameter else 300  # μm
        self.zp_na = zp_na if zp_na else 0.1
        self.cond_inner = cond_inner if cond_inner else 450  # μm
        self.cond_outer = cond_outer if cond_outer else 750  # μm
        self.draw_schematic()

    def draw_schematic(self):
        """Draw the instrument schematic"""
        self.scene.clear()

        # Colors
        beam_color = QColor(255, 200, 100, 180)  # Orange beam
        component_color = QColor(100, 150, 255)  # Blue components
        text_color = QColor(224, 224, 224)  # Light gray text
        distance_color = QColor(150, 150, 150)  # Gray for distances

        # Layout parameters - Fixed to prevent shifting
        width = 750
        height = 200
        y_center = height / 2

        # Set fixed scene rectangle to prevent shifting
        self.scene.setSceneRect(0, 0, width, height)

        # Calculate component positions based on actual distances (RIGHT to LEFT)
        # Use non-linear scaling to better represent both short and long distances

        # Available horizontal space for the layout
        available_width = 650  # pixels
        margin_left = 50
        margin_right = 50

        # Assign proportional pixel widths to each section
        # Give more visual space to the short distances
        source_cond_width = 100  # Fixed space for source to condenser
        cond_sample_width = 80   # Fixed space for condenser to sample
        sample_zp_width = 120    # Space for sample to ZP (focal length)
        zp_camera_width = 350    # Remaining space for ZP to camera (longest distance)

        # Position components from RIGHT to LEFT
        x_source = margin_right + available_width  # Rightmost
        x_condenser = x_source - source_cond_width
        x_sample = x_condenser - cond_sample_width
        x_zp = x_sample - sample_zp_width
        x_camera = margin_left  # Leftmost
        # Phase ring is at back focal plane (one focal length from ZP)
        # Place it proportionally between ZP and camera
        x_phase_ring = x_zp - (zp_camera_width * 0.25)

        # Calculate component sizes first (needed for ray tracing)
        # Condenser size
        cond_half_outer = 22 + ((self.cond_outer - 450) / 300) * 10
        cond_half_outer = max(20, min(35, cond_half_outer))
        cond_half_inner = 15 + ((self.cond_inner - 450) / 300) * 7
        cond_half_inner = max(13, min(25, cond_half_inner))

        # Zone plate size
        zp_radius = 12 + ((self.zp_diameter - 150) / 150) * 10
        zp_radius = max(10, min(25, zp_radius))

        # Draw beam path with ray tracing
        # Central beam (chief ray)
        pen_beam_main = QPen(beam_color, 8)
        self.scene.addLine(x_source, y_center, x_camera, y_center, pen_beam_main)

        # Ray tracing - show beam focusing (dynamically scaled to optics parameters)
        pen_ray = QPen(QColor(255, 200, 100, 120), 2)

        import math
        num_rays = 5

        # Scale factors for ray tracing - use consistent visual representation
        # Match the component visual sizes we calculated above
        source_divergence = cond_half_outer * 1.2  # Slightly wider than condenser outer
        condenser_aperture = cond_half_outer * 0.9  # At condenser

        # Sample spot size based on condenser convergence
        sample_spot = 12  # Small focused spot at sample

        # Zone plate aperture - use the actual ZP radius we calculated
        zp_aperture = zp_radius * 0.9  # Slightly smaller than ZP diameter

        # Phase ring - focused spot based on ZP NA
        # Higher NA (0.15) = smaller spot (~8px), Lower NA (0.05) = larger spot (~14px)
        phase_spot = max(6, min(15, 16 - (self.zp_na * 1000) * 0.5))

        # Camera - final image spot
        camera_focus = 15  # Slight divergence at detector

        for i in range(-num_rays, num_rays + 1):
            if i == 0:
                continue  # Skip central ray (already drawn)

            # Source divergence
            y_start = y_center + (i / num_rays) * source_divergence

            # Condenser focuses rays
            y_cond = y_center + (i / num_rays) * condenser_aperture

            # Draw ray from source to condenser
            self.scene.addLine(x_source, y_start, x_condenser, y_cond, pen_ray)

            # After condenser - converging to sample
            y_sample = y_center + (i / num_rays) * sample_spot
            self.scene.addLine(x_condenser, y_cond, x_sample, y_sample, pen_ray)

            # After sample - diverging towards zone plate
            y_zp = y_center + (i / num_rays) * zp_aperture
            self.scene.addLine(x_sample, y_sample, x_zp, y_zp, pen_ray)

            # Zone plate focuses rays to phase ring at back focal plane
            y_phase = y_center + (i / num_rays) * phase_spot
            self.scene.addLine(x_zp, y_zp, x_phase_ring, y_phase, pen_ray)

            # From phase ring to camera (slight divergence)
            y_camera = y_center + (i / num_rays) * camera_focus
            self.scene.addLine(x_phase_ring, y_phase, x_camera, y_camera, pen_ray)

        # Draw X-ray source (on the right)
        pen_component = QPen(component_color, 2)
        brush_component = QBrush(component_color)

        # Source (point with rays)
        self.scene.addEllipse(x_source - 8, y_center - 8, 16, 16, pen_component, brush_component)
        # Add radiating lines to show it's a source
        for angle in [-20, -10, 0, 10, 20]:
            import math
            rad = math.radians(angle)
            x_end = x_source + 20 * math.cos(rad)
            y_end = y_center + 20 * math.sin(rad)
            self.scene.addLine(x_source, y_center, x_end, y_end, QPen(beam_color, 2))

        source_text = self.scene.addText("X-ray\nSource", QFont("Arial", 9))
        source_text.setDefaultTextColor(text_color)
        source_text.setPos(x_source - 25, y_center - 45)

        # Condenser (trapezoid shape - focusing from right, scaled by aperture)
        # Sizes already calculated above
        condenser_poly = QPolygonF([
            QPointF(x_condenser + 15, y_center - cond_half_outer),
            QPointF(x_condenser - 15, y_center - cond_half_inner),
            QPointF(x_condenser - 15, y_center + cond_half_inner),
            QPointF(x_condenser + 15, y_center + cond_half_outer)
        ])
        self.scene.addPolygon(condenser_poly, pen_component, brush_component)
        cond_text = self.scene.addText(f"Condenser\n{self.condenser_name}", QFont("Arial", 8))
        cond_text.setDefaultTextColor(text_color)
        cond_text.setPos(x_condenser - 35, y_center + cond_half_outer + 5)

        # Sample (small rectangle)
        self.scene.addRect(x_sample - 3, y_center - 20, 6, 40, pen_component, brush_component)
        sample_text = self.scene.addText("Sample", QFont("Arial", 9))
        sample_text.setDefaultTextColor(text_color)
        sample_text.setPos(x_sample - 25, y_center + 25)

        # Zone Plate (thin circle with rings, scaled by diameter)
        # Radius already calculated above
        pen_zp = QPen(component_color, 3)
        self.scene.addEllipse(x_zp - zp_radius, y_center - zp_radius,
                            zp_radius * 2, zp_radius * 2, pen_zp)
        # Add inner rings to show zone structure
        self.scene.addEllipse(x_zp - zp_radius*0.65, y_center - zp_radius*0.65,
                            zp_radius*1.3, zp_radius*1.3, QPen(component_color, 1))
        self.scene.addEllipse(x_zp - zp_radius*0.33, y_center - zp_radius*0.33,
                            zp_radius*0.66, zp_radius*0.66, QPen(component_color, 1))
        zp_text = self.scene.addText(f"Zone Plate\n{self.zp_name}", QFont("Arial", 8))
        zp_text.setDefaultTextColor(text_color)
        zp_text.setPos(x_zp - 30, y_center - zp_radius - 35)

        # Phase Ring (annular ring at back focal plane of ZP)
        pen_phase = QPen(QColor(200, 100, 200), 3)  # Purple for phase ring
        self.scene.addEllipse(x_phase_ring - 18, y_center - 18, 36, 36, pen_phase)
        self.scene.addEllipse(x_phase_ring - 10, y_center - 10, 20, 20, pen_phase)
        phase_text = self.scene.addText("Phase Ring\n(Back Focal)", QFont("Arial", 7))
        phase_text.setDefaultTextColor(QColor(200, 150, 200))
        phase_text.setPos(x_phase_ring - 30, y_center + 25)

        # Camera (rectangle with chip - on the left)
        self.scene.addRect(x_camera - 20, y_center - 30, 25, 60, pen_component, brush_component)
        self.scene.addRect(x_camera - 13, y_center - 15, 11, 30, QPen(Qt.white, 1), QBrush(Qt.white))
        cam_text = self.scene.addText(f"Camera\n{self.camera_name[:12]}...", QFont("Arial", 7))
        cam_text.setDefaultTextColor(text_color)
        cam_text.setPos(x_camera - 30, y_center + 35)

        # Draw distance annotations
        pen_distance = QPen(distance_color, 1, Qt.DashLine)
        font_distance = QFont("Arial", 7)

        # Source to Condenser
        if abs(x_source - x_condenser) > 40:  # Only show if there's space
            y_dist = y_center - 50
            self.scene.addLine(x_condenser, y_dist, x_source, y_dist, pen_distance)
            dist_text = self.scene.addText(f"{self.source_to_condenser:.0f} mm", font_distance)
            dist_text.setDefaultTextColor(distance_color)
            dist_text.setPos((x_source + x_condenser) / 2 - 20, y_dist - 15)

        # Condenser to Sample
        if abs(x_condenser - x_sample) > 40 and self.condenser_to_sample > 0:
            y_dist = y_center + 55
            self.scene.addLine(x_sample, y_dist, x_condenser, y_dist, pen_distance)
            dist_text = self.scene.addText(f"{self.condenser_to_sample:.0f} mm", font_distance)
            dist_text.setDefaultTextColor(distance_color)
            dist_text.setPos((x_condenser + x_sample) / 2 - 20, y_dist + 5)

        # Sample to ZP (focal length)
        if self.sample_to_zp > 0 and abs(x_sample - x_zp) > 40:
            y_dist = y_center - 60
            self.scene.addLine(x_zp, y_dist, x_sample, y_dist, pen_distance)
            dist_text = self.scene.addText(f"f = {self.sample_to_zp:.2f} mm", font_distance)
            dist_text.setDefaultTextColor(distance_color)
            dist_text.setPos((x_sample + x_zp) / 2 - 25, y_dist - 15)

        # ZP to Camera (show full distance)
        if abs(x_zp - x_camera) > 60:
            y_dist = y_center + 50
            self.scene.addLine(x_camera, y_dist, x_zp, y_dist, pen_distance)
            # Show in meters if > 1000mm
            if self.zp_to_camera >= 1000:
                dist_text = self.scene.addText(f"{self.zp_to_camera/1000:.1f} m", font_distance)
            else:
                dist_text = self.scene.addText(f"{self.zp_to_camera:.0f} mm", font_distance)
            dist_text.setDefaultTextColor(distance_color)
            dist_text.setPos((x_zp + x_camera) / 2 - 20, y_dist + 5)

        # Add arrow showing beam direction (right to left)
        arrow_y = y_center - 35
        arrow_color = QColor(255, 200, 100, 200)
        self.scene.addLine(x_source - 30, arrow_y, x_camera + 30, arrow_y, QPen(arrow_color, 2))
        # Arrow head pointing left
        arrow_head = QPolygonF([
            QPointF(x_camera + 30, arrow_y),
            QPointF(x_camera + 40, arrow_y - 5),
            QPointF(x_camera + 40, arrow_y + 5)
        ])
        self.scene.addPolygon(arrow_head, QPen(arrow_color, 1), QBrush(arrow_color))

        # Set scene rectangle
        self.scene.setSceneRect(0, 0, width, height)
        self.view.fitInView(self.scene.sceneRect(), Qt.KeepAspectRatio)


class OpticsManagerDialog(QDialog):
    """Dialog for managing optics configurations (add/edit/delete)"""

    def __init__(self, parent, optics_type, optics_dict, title):
        super().__init__(parent)
        self.optics_type = optics_type
        self.optics_dict = optics_dict.copy()
        self.setWindowTitle(title)
        self.setMinimumSize(800, 500)
        self.setup_ui()

    def setup_ui(self):
        layout = QVBoxLayout()

        # Table to display optics
        self.table = QTableWidget()
        self.table.setSelectionBehavior(QTableWidget.SelectRows)
        self.table.setSelectionMode(QTableWidget.SingleSelection)
        layout.addWidget(self.table)

        # Buttons
        button_layout = QHBoxLayout()

        self.add_btn = QPushButton("Add New")
        self.add_btn.clicked.connect(self.add_optic)
        button_layout.addWidget(self.add_btn)

        self.edit_btn = QPushButton("Edit Selected")
        self.edit_btn.clicked.connect(self.edit_optic)
        button_layout.addWidget(self.edit_btn)

        self.delete_btn = QPushButton("Delete Selected")
        self.delete_btn.clicked.connect(self.delete_optic)
        button_layout.addWidget(self.delete_btn)

        button_layout.addStretch()
        layout.addLayout(button_layout)

        # Dialog buttons
        button_box = QDialogButtonBox(QDialogButtonBox.Ok |
                                     QDialogButtonBox.Cancel)
        button_box.accepted.connect(self.accept)
        button_box.rejected.connect(self.reject)
        layout.addWidget(button_box)

        self.setLayout(layout)
        self.populate_table()

    def populate_table(self):
        """Populate table with current optics data"""
        if not self.optics_dict:
            return

        # Get parameter names from first entry
        first_key = list(self.optics_dict.keys())[0]
        param_names = list(self.optics_dict[first_key].keys())

        self.table.setColumnCount(len(param_names) + 1)
        self.table.setHorizontalHeaderLabels(["Name"] + param_names)
        self.table.setRowCount(len(self.optics_dict))

        for row, (name, params) in enumerate(self.optics_dict.items()):
            # Name column
            self.table.setItem(row, 0, QTableWidgetItem(name))

            # Parameter columns
            for col, param_name in enumerate(param_names):
                value = params.get(param_name, "")
                self.table.setItem(row, col + 1, QTableWidgetItem(str(value)))

        self.table.horizontalHeader().setSectionResizeMode(QHeaderView.Stretch)

    def add_optic(self):
        """Add a new optic configuration"""
        dialog = OpticEditorDialog(self, self.optics_type, None, None)
        if dialog.exec_() == QDialog.Accepted:
            name, params = dialog.get_data()
            if name in self.optics_dict:
                QMessageBox.warning(self, "Duplicate Name",
                                  f"An optic named '{name}' already exists.")
                return
            self.optics_dict[name] = params
            self.populate_table()

    def edit_optic(self):
        """Edit selected optic"""
        current_row = self.table.currentRow()
        if current_row < 0:
            QMessageBox.warning(self, "No Selection", "Please select an optic to edit.")
            return

        name = self.table.item(current_row, 0).text()
        params = self.optics_dict[name]

        dialog = OpticEditorDialog(self, self.optics_type, name, params)
        if dialog.exec_() == QDialog.Accepted:
            new_name, new_params = dialog.get_data()

            # Remove old entry if name changed
            if new_name != name:
                del self.optics_dict[name]

            self.optics_dict[new_name] = new_params
            self.populate_table()

    def delete_optic(self):
        """Delete selected optic"""
        current_row = self.table.currentRow()
        if current_row < 0:
            QMessageBox.warning(self, "No Selection", "Please select an optic to delete.")
            return

        name = self.table.item(current_row, 0).text()

        reply = QMessageBox.question(self, "Confirm Delete",
                                    f"Are you sure you want to delete '{name}'?",
                                    QMessageBox.Yes |
                                    QMessageBox.No)

        if reply == QMessageBox.Yes:
            del self.optics_dict[name]
            self.populate_table()

    def get_optics_dict(self):
        """Return the modified optics dictionary"""
        return self.optics_dict


class OpticEditorDialog(QDialog):
    """Dialog for editing individual optic parameters"""

    # Define parameter fields for each optics type
    PARAM_DEFINITIONS = {
        "zone_plates": [
            ("drn", "Outer Ring (nm)", "float"),
            ("diameter", "Diameter (μm)", "float"),
            ("smallest_feature", "Smallest Feature (nm)", "float"),
            ("efficiency", "Efficiency (%)", "float"),
            ("thickness", "Thickness (nm)", "float")
        ],
        "condensers": [
            ("source_dist", "Source Distance (m)", "float"),
            ("inner_dia", "Inner Diameter (μm)", "float"),
            ("outer_dia", "Outer Diameter (μm)", "float"),
            ("length", "Length (mm)", "float"),
            ("focal_length", "Focal Length (mm)", "float"),
            ("na_tip", "NA Tip", "float"),
            ("position_offset", "Position Offset (mm)", "float")
        ],
        "cameras": [
            ("pixel_size", "Pixel Size (μm)", "float"),
            ("pixels_v", "Pixels Vertical", "int"),
            ("pixels_h", "Pixels Horizontal", "int"),
            ("scintillator", "Scintillator", "str"),
            ("efficiency", "Efficiency (%)", "float")
        ],
        "optical_magnifications": [
            ("magnification", "Magnification", "float"),
            ("na", "Numerical Aperture", "float"),
            ("working_distance", "Working Distance (mm)", "float")
        ]
    }

    def __init__(self, parent, optics_type, name=None, params=None):
        super().__init__(parent)
        self.optics_type = optics_type
        self.original_name = name
        self.setWindowTitle("Edit Optic" if name else "Add New Optic")
        self.setMinimumWidth(400)

        self.setup_ui(name, params)

    def setup_ui(self, name, params):
        layout = QVBoxLayout()

        # Name field
        name_layout = QHBoxLayout()
        name_layout.addWidget(QLabel("Name:"))
        self.name_edit = QLineEdit(name or "")
        name_layout.addWidget(self.name_edit)
        layout.addLayout(name_layout)

        # Parameter fields
        self.param_edits = {}
        param_defs = self.PARAM_DEFINITIONS.get(self.optics_type, [])

        grid = QGridLayout()
        for row, (key, label, data_type) in enumerate(param_defs):
            grid.addWidget(QLabel(label + ":"), row, 0)

            edit = QLineEdit()
            if params and key in params:
                edit.setText(str(params[key]))

            self.param_edits[key] = (edit, data_type)
            grid.addWidget(edit, row, 1)

        layout.addLayout(grid)

        # Dialog buttons
        button_box = QDialogButtonBox(QDialogButtonBox.Ok |
                                     QDialogButtonBox.Cancel)
        button_box.accepted.connect(self.validate_and_accept)
        button_box.rejected.connect(self.reject)
        layout.addWidget(button_box)

        self.setLayout(layout)

    def validate_and_accept(self):
        """Validate input and accept dialog"""
        name = self.name_edit.text().strip()
        if not name:
            QMessageBox.warning(self, "Invalid Input", "Name cannot be empty.")
            return

        # Validate parameters
        try:
            for key, (edit, data_type) in self.param_edits.items():
                value = edit.text().strip()
                if data_type == "float":
                    float(value)
                elif data_type == "int":
                    int(value)
        except ValueError:
            QMessageBox.warning(self, "Invalid Input",
                              f"Invalid value for {key}. Please check all fields.")
            return

        self.accept()

    def get_data(self):
        """Return the name and parameters dictionary"""
        name = self.name_edit.text().strip()
        params = {}

        for key, (edit, data_type) in self.param_edits.items():
            value = edit.text().strip()
            if data_type == "float":
                params[key] = float(value)
            elif data_type == "int":
                params[key] = int(value)
            else:
                params[key] = value

        return name, params


class ModernOpticsCalculator(QMainWindow):
    def __init__(self):
        super().__init__()
        self.setWindowTitle("X-ray Optics Calculator")
        self.setGeometry(100, 100, 1200, 900)

        # Configuration file
        self.config_file = "optics_config.json"

        # Default configurations
        self.default_configs = {
            "zone_plates": {
                "60nm ZP": {"drn": 60, "diameter": 180, "smallest_feature": 30, "efficiency": 20, "thickness": 500},
                "50nm ZP": {"drn": 50, "diameter": 180, "smallest_feature": 25, "efficiency": 25, "thickness": 500},
                "40nm ZP": {"drn": 40, "diameter": 180, "smallest_feature": 20, "efficiency": 30, "thickness": 500},
                "30nm ZP": {"drn": 30, "diameter": 300, "smallest_feature": 30, "efficiency": 35, "thickness": 500},
                "16nm ZP": {"drn": 16, "diameter": 150, "smallest_feature": 16, "efficiency": 35, "thickness": 500}
            },
            "condensers": {
                "Sigray": {"source_dist": 35, "inner_dia": 450, "outer_dia": 750,
                          "length": 80, "focal_length": 43.2, "na_tip": 5.208, "position_offset": 100},
                "Sigray2": {"source_dist": 35, "inner_dia": 475, "outer_dia": 749,
                           "length": 145, "focal_length": 96.9, "na_tip": 2.451, "position_offset": 150}
            },
            "cameras": {
                "ORX-10G-310S9M": {"pixel_size": 3.45, "pixels_v": 6464, "pixels_h": 4852,
                                   "scintillator": "GGG:Ce", "efficiency": 15},
                "Generic Camera": {"pixel_size": 5.0, "pixels_v": 1024, "pixels_h": 1024,
                                 "scintillator": "CsI:Tl", "efficiency": 10}
            },
            "optical_magnifications": {
                "2x Objective": {"magnification": 2.0, "na": 0.055, "working_distance": 31.0},
                "5x Objective": {"magnification": 5.0, "na": 0.13, "working_distance": 17.0},
                "9x Objective": {"magnification": 9.0, "na": 0.28, "working_distance": 34.0},
            },
            "user_parameters": {
                "camera_distance": 3500.0,
                "binning": 2,
                "exposure_time": 0.001,
                "sample_detector_distance": 3500.0,
                "beam_current": 200.0,
                "source_distance": 35000.0  # mm (35m default)
            }
        }

        # Load configuration
        self.load_config()

        # Setup UI
        self.setup_ui()

        # Apply modern styling
        self.apply_styling()

        # Initial calculation
        self.calculate()

    def load_config(self):
        """Load configuration from JSON file"""
        try:
            if os.path.exists(self.config_file):
                with open(self.config_file, 'r') as f:
                    loaded_config = json.load(f)
                self.configs = self.merge_configs(self.default_configs, loaded_config)
            else:
                self.configs = self.default_configs.copy()
                self.save_config()
        except Exception as e:
            print(f"Error loading config: {e}")
            self.configs = self.default_configs.copy()

    def merge_configs(self, default, loaded):
        """Merge loaded config with defaults"""
        result = default.copy()
        for key, value in loaded.items():
            if key in result and isinstance(value, dict):
                result[key] = self.merge_configs(result[key], value)
            else:
                result[key] = value
        return result

    def save_config(self):
        """Save current configuration to JSON file"""
        try:
            # Update user parameters from UI
            self.configs["user_parameters"]["camera_distance"] = float(self.camera_distance_edit.text())
            self.configs["user_parameters"]["binning"] = int(self.binning_edit.text())
            self.configs["user_parameters"]["exposure_time"] = float(self.exposure_time_edit.text())
            self.configs["user_parameters"]["sample_detector_distance"] = float(self.sample_det_distance_edit.text())
            self.configs["user_parameters"]["beam_current"] = float(self.beam_current_edit.text())
            self.configs["user_parameters"]["source_distance"] = float(self.source_distance_edit.text())

            with open(self.config_file, 'w') as f:
                json.dump(self.configs, f, indent=2)

            self.status_label.setText("✓ Configuration saved successfully")
            return True
        except ValueError as e:
            self.status_label.setText(f"❌ Invalid value: {e}")
            return False
        except Exception as e:
            self.status_label.setText(f"❌ Save error: {e}")
            return False

    def setup_ui(self):
        """Setup the main UI"""
        # Central widget
        central_widget = QWidget()
        self.setCentralWidget(central_widget)

        main_layout = QVBoxLayout()
        central_widget.setLayout(main_layout)

        # Title
        title = QLabel("X-ray Optics Calculator")
        title.setFont(QFont("Arial", 24, QFont.Bold))
        title.setAlignment(Qt.AlignCenter)
        main_layout.addWidget(title)

        # Tab widget
        tabs = QTabWidget()
        main_layout.addWidget(tabs)

        # Calculator tab
        calc_widget = QWidget()
        self.setup_calculator_tab(calc_widget)
        tabs.addTab(calc_widget, "Calculator")

        # Configuration tab
        config_widget = QWidget()
        self.setup_config_tab(config_widget)
        tabs.addTab(config_widget, "Configuration")

        # Status bar
        self.status_label = QLabel("Ready - Configuration loaded")
        self.status_label.setObjectName("statusLabel")
        main_layout.addWidget(self.status_label)

    def setup_calculator_tab(self, parent):
        """Setup the calculator tab"""
        layout = QVBoxLayout()
        parent.setLayout(layout)

        # Schematic diagram
        schematic_group = QGroupBox("Instrument Schematic")
        schematic_layout = QVBoxLayout()
        self.schematic = InstrumentSchematic()
        schematic_layout.addWidget(self.schematic)
        schematic_group.setLayout(schematic_layout)
        layout.addWidget(schematic_group)

        # Input section
        self.create_input_section(layout)

        # User parameters section
        self.create_user_params_section(layout)

        # Results section
        self.create_results_section(layout)

        # Control buttons
        self.create_control_section(layout)

    def create_input_section(self, layout):
        """Create input parameters section"""
        group = QGroupBox("Input Parameters")
        group_layout = QVBoxLayout()

        # Energy input
        energy_layout = QHBoxLayout()
        energy_layout.addWidget(QLabel("Energy (eV):"))
        self.energy_edit = QLineEdit("8000")
        self.energy_edit.textChanged.connect(self.calculate)
        energy_layout.addWidget(self.energy_edit)

        energy_layout.addWidget(QLabel("Wavelength:"))
        self.wavelength_label = QLabel()
        self.wavelength_label.setStyleSheet("color: #3498db; font-weight: bold;")
        energy_layout.addWidget(self.wavelength_label)
        energy_layout.addWidget(QLabel("nm"))
        energy_layout.addStretch()
        group_layout.addLayout(energy_layout)

        # Selections
        select_layout = QHBoxLayout()

        # Zone plate
        zp_layout = QVBoxLayout()
        zp_layout.addWidget(QLabel("Zone Plate:"))
        self.zp_combo = QComboBox()
        self.zp_combo.addItems(list(self.configs["zone_plates"].keys()))
        self.zp_combo.setCurrentText("30nm ZP")
        self.zp_combo.currentTextChanged.connect(self.calculate)
        zp_layout.addWidget(self.zp_combo)
        select_layout.addLayout(zp_layout)

        # Condenser
        cond_layout = QVBoxLayout()
        cond_layout.addWidget(QLabel("Condenser:"))
        self.condenser_combo = QComboBox()
        self.condenser_combo.addItems(list(self.configs["condensers"].keys()))
        self.condenser_combo.setCurrentText("Sigray")
        self.condenser_combo.currentTextChanged.connect(self.calculate)
        cond_layout.addWidget(self.condenser_combo)
        select_layout.addLayout(cond_layout)

        # Camera
        cam_layout = QVBoxLayout()
        cam_layout.addWidget(QLabel("Camera:"))
        self.camera_combo = QComboBox()
        self.camera_combo.addItems(list(self.configs["cameras"].keys()))
        self.camera_combo.setCurrentText("ORX-10G-310S9M")
        self.camera_combo.currentTextChanged.connect(self.calculate)
        cam_layout.addWidget(self.camera_combo)
        select_layout.addLayout(cam_layout)

        # Optical Magnification
        opt_layout = QVBoxLayout()
        opt_layout.addWidget(QLabel("Optical Mag:"))
        self.optical_mag_combo = QComboBox()
        self.optical_mag_combo.addItems(list(self.configs["optical_magnifications"].keys()))
        self.optical_mag_combo.setCurrentText("2x Objective")
        self.optical_mag_combo.currentTextChanged.connect(self.calculate)
        opt_layout.addWidget(self.optical_mag_combo)
        select_layout.addLayout(opt_layout)

        group_layout.addLayout(select_layout)
        group.setLayout(group_layout)
        layout.addWidget(group)

    def create_user_params_section(self, layout):
        """Create user parameters section"""
        group = QGroupBox("User Parameters")
        grid = QGridLayout()

        # Load saved parameters
        user_params = self.configs["user_parameters"]

        # Camera Distance
        grid.addWidget(QLabel("Camera Distance (mm):"), 0, 0)
        self.camera_distance_edit = QLineEdit(str(user_params["camera_distance"]))
        self.camera_distance_edit.textChanged.connect(self.calculate)
        grid.addWidget(self.camera_distance_edit, 0, 1)

        # Binning
        grid.addWidget(QLabel("Binning:"), 0, 2)
        self.binning_edit = QLineEdit(str(user_params["binning"]))
        self.binning_edit.textChanged.connect(self.calculate)
        grid.addWidget(self.binning_edit, 0, 3)

        # Exposure Time
        grid.addWidget(QLabel("Exposure Time (s):"), 0, 4)
        self.exposure_time_edit = QLineEdit(str(user_params["exposure_time"]))
        grid.addWidget(self.exposure_time_edit, 0, 5)

        # Sample-Detector Distance
        grid.addWidget(QLabel("Sample-Det Distance (mm):"), 1, 0)
        self.sample_det_distance_edit = QLineEdit(str(user_params["sample_detector_distance"]))
        grid.addWidget(self.sample_det_distance_edit, 1, 1)

        # Beam Current
        grid.addWidget(QLabel("Beam Current (mA):"), 1, 2)
        self.beam_current_edit = QLineEdit(str(user_params["beam_current"]))
        grid.addWidget(self.beam_current_edit, 1, 3)

        # Source Distance
        grid.addWidget(QLabel("Source Distance (mm):"), 1, 4)
        self.source_distance_edit = QLineEdit(str(user_params.get("source_distance", 35000.0)))
        self.source_distance_edit.textChanged.connect(self.calculate)
        grid.addWidget(self.source_distance_edit, 1, 5)

        # Save button
        save_btn = QPushButton("Save Config")
        save_btn.clicked.connect(self.save_config_with_feedback)
        grid.addWidget(save_btn, 2, 0, 1, 2)

        group.setLayout(grid)
        layout.addWidget(group)

    def create_results_section(self, layout):
        """Create results display section"""
        results_layout = QHBoxLayout()

        # Zone Plate Results
        self.zp_results = self.create_result_card("Zone Plate Calculations", [
            ("Diameter (μm):", "zp_diameter"),
            ("Outer Ring (nm):", "zp_drn"),
            ("Smallest Feature (nm):", "zp_smallest"),
            ("Focal Length (mm):", "zp_focal"),
            ("Numerical Aperture (mrad):", "zp_na"),
            ("Number of Zones:", "zp_zones"),
            ("Motor Position (mm):", "zp_position", True)
        ])
        results_layout.addWidget(self.zp_results)

        # Camera Results
        self.cam_results = self.create_result_card("Camera & Imaging", [
            ("Pixel Size (μm):", "cam_pixel"),
            ("Resolution (px):", "cam_pixels"),
            ("Effective Resolution (px):", "cam_eff_pixels"),
            ("Optical Magnification (×):", "opt_mag"),
            ("Total Magnification (×):", "total_mag"),
            ("Effective Pixel (nm):", "cam_eff_pixel"),
            ("Field of View (μm):", "cam_fov"),
            ("Distance (mm):", "cam_distance_user", True)
        ])
        results_layout.addWidget(self.cam_results)

        # Condenser Results
        self.cond_results = self.create_result_card("Condenser", [
            ("Type:", "cond_type"),
            ("Source Distance (m):", "cond_source"),
            ("Inner Diameter (μm):", "cond_inner"),
            ("Outer Diameter (μm):", "cond_outer"),
            ("Length (mm):", "cond_length"),
            ("Focal Length (mm):", "cond_focal"),
            ("Motor Position (mm):", "cond_position", True)
        ])
        results_layout.addWidget(self.cond_results)

        layout.addLayout(results_layout)

    def create_result_card(self, title, fields):
        """Create a result card widget"""
        group = QGroupBox(title)
        layout = QVBoxLayout()

        self.result_labels = getattr(self, 'result_labels', {})

        for field_data in fields:
            label_text = field_data[0]
            field_name = field_data[1]
            is_motor = field_data[2] if len(field_data) > 2 else False

            field_layout = QHBoxLayout()
            field_layout.addWidget(QLabel(label_text))

            value_label = QLabel("---")
            value_label.setStyleSheet(f"color: {'#e74c3c' if is_motor else '#3498db'}; font-weight: bold;")
            field_layout.addWidget(value_label)
            field_layout.addStretch()

            self.result_labels[field_name] = value_label
            layout.addLayout(field_layout)

        group.setLayout(layout)
        return group

    def create_control_section(self, layout):
        """Create control buttons"""
        button_layout = QHBoxLayout()
        button_layout.addStretch()

        calc_btn = QPushButton("Calculate")
        calc_btn.clicked.connect(self.calculate)
        button_layout.addWidget(calc_btn)

        reset_btn = QPushButton("Reset")
        reset_btn.clicked.connect(self.reset)
        button_layout.addWidget(reset_btn)

        export_btn = QPushButton("Export Results")
        export_btn.clicked.connect(self.export_results)
        button_layout.addWidget(export_btn)

        button_layout.addStretch()
        layout.addLayout(button_layout)

    def setup_config_tab(self, parent):
        """Setup configuration tab"""
        layout = QVBoxLayout()
        parent.setLayout(layout)

        title = QLabel("Optics Management")
        title.setFont(QFont("Arial", 16, QFont.Bold))
        title.setAlignment(Qt.AlignCenter)
        layout.addWidget(title)

        # Buttons for managing different optics types
        grid = QGridLayout()

        # Zone Plates
        zp_btn = QPushButton("Manage Zone Plates")
        zp_btn.clicked.connect(lambda: self.manage_optics("zone_plates", "Zone Plates Manager"))
        grid.addWidget(zp_btn, 0, 0)

        # Condensers
        cond_btn = QPushButton("Manage Condensers")
        cond_btn.clicked.connect(lambda: self.manage_optics("condensers", "Condensers Manager"))
        grid.addWidget(cond_btn, 0, 1)

        # Cameras
        cam_btn = QPushButton("Manage Cameras")
        cam_btn.clicked.connect(lambda: self.manage_optics("cameras", "Cameras Manager"))
        grid.addWidget(cam_btn, 1, 0)

        # Optical Magnifications
        opt_btn = QPushButton("Manage Optical Magnifications")
        opt_btn.clicked.connect(lambda: self.manage_optics("optical_magnifications", "Optical Magnifications Manager"))
        grid.addWidget(opt_btn, 1, 1)

        layout.addLayout(grid)
        layout.addSpacing(20)

        # Config file operations
        file_layout = QHBoxLayout()
        file_layout.addStretch()

        export_btn = QPushButton("Export Config")
        export_btn.clicked.connect(self.export_config)
        file_layout.addWidget(export_btn)

        import_btn = QPushButton("Import Config")
        import_btn.clicked.connect(self.import_config)
        file_layout.addWidget(import_btn)

        reset_btn = QPushButton("Reset to Defaults")
        reset_btn.clicked.connect(self.reset_to_defaults)
        file_layout.addWidget(reset_btn)

        file_layout.addStretch()
        layout.addLayout(file_layout)

        layout.addStretch()

    def manage_optics(self, optics_type, title):
        """Open optics manager dialog"""
        dialog = OpticsManagerDialog(self, optics_type, self.configs[optics_type], title)
        if dialog.exec_() == QDialog.Accepted:
            self.configs[optics_type] = dialog.get_optics_dict()
            self.update_combo_boxes()
            self.save_config()
            QMessageBox.information(self, "Success", "Optics updated successfully!")

    def update_combo_boxes(self):
        """Update all combo boxes with new configurations"""
        # Save current selections
        current_zp = self.zp_combo.currentText()
        current_cond = self.condenser_combo.currentText()
        current_cam = self.camera_combo.currentText()
        current_opt = self.optical_mag_combo.currentText()

        # Update zone plates
        self.zp_combo.clear()
        self.zp_combo.addItems(list(self.configs["zone_plates"].keys()))
        if current_zp in self.configs["zone_plates"]:
            self.zp_combo.setCurrentText(current_zp)

        # Update condensers
        self.condenser_combo.clear()
        self.condenser_combo.addItems(list(self.configs["condensers"].keys()))
        if current_cond in self.configs["condensers"]:
            self.condenser_combo.setCurrentText(current_cond)

        # Update cameras
        self.camera_combo.clear()
        self.camera_combo.addItems(list(self.configs["cameras"].keys()))
        if current_cam in self.configs["cameras"]:
            self.camera_combo.setCurrentText(current_cam)

        # Update optical magnifications
        self.optical_mag_combo.clear()
        self.optical_mag_combo.addItems(list(self.configs["optical_magnifications"].keys()))
        if current_opt in self.configs["optical_magnifications"]:
            self.optical_mag_combo.setCurrentText(current_opt)

    def calculate(self):
        """Perform calculations"""
        try:
            # Get energy and calculate wavelength
            energy = float(self.energy_edit.text())
            wavelength = 1240 / energy
            self.wavelength_label.setText(f"{wavelength:.3f}")

            # Get configurations
            zp_config = self.configs["zone_plates"][self.zp_combo.currentText()]
            camera_config = self.configs["cameras"][self.camera_combo.currentText()]
            cond_config = self.configs["condensers"][self.condenser_combo.currentText()]
            opt_mag_config = self.configs["optical_magnifications"][self.optical_mag_combo.currentText()]

            # Get user parameters
            camera_distance = float(self.camera_distance_edit.text())
            binning = int(self.binning_edit.text())
            source_distance = float(self.source_distance_edit.text())

            # Zone plate calculations
            diameter = zp_config["diameter"]
            drn = zp_config["drn"]
            smallest_feature = zp_config["smallest_feature"]

            focal_length = (diameter * drn) / wavelength / 1000
            na = (1000 * wavelength) / (2 * drn)
            num_zones = (1000 * diameter) / (4 * drn)
            zp_position = focal_length

            # Camera calculations
            pixel_size = camera_config["pixel_size"]
            pixels_v = camera_config["pixels_v"]
            pixels_h = camera_config["pixels_h"]

            eff_pixels_v = pixels_v // binning
            eff_pixels_h = pixels_h // binning
            eff_pixel_size = pixel_size * binning

            optical_magnification = opt_mag_config["magnification"]
            zp_magnification = camera_distance / focal_length
            total_magnification = zp_magnification * optical_magnification

            effective_pixel = (eff_pixel_size * 1000) / total_magnification

            chip_size_v = eff_pixels_v * eff_pixel_size / 1000
            chip_size_h = eff_pixels_h * eff_pixel_size / 1000
            fov_v = (chip_size_v * 1000) / total_magnification
            fov_h = (chip_size_h * 1000) / total_magnification

            # Update displays
            self.result_labels["zp_diameter"].setText(f"{diameter}")
            self.result_labels["zp_drn"].setText(f"{drn}")
            self.result_labels["zp_smallest"].setText(f"{smallest_feature}")
            self.result_labels["zp_focal"].setText(f"{focal_length:.2f}")
            self.result_labels["zp_na"].setText(f"{na:.2f}")
            self.result_labels["zp_zones"].setText(f"{num_zones:.0f}")
            self.result_labels["zp_position"].setText(f"{zp_position:.2f}")

            self.result_labels["cam_pixel"].setText(f"{pixel_size}")
            self.result_labels["cam_pixels"].setText(f"{pixels_v}×{pixels_h}")
            self.result_labels["cam_eff_pixels"].setText(f"{eff_pixels_v}×{eff_pixels_h}")
            self.result_labels["opt_mag"].setText(f"{optical_magnification:.1f}")
            self.result_labels["total_mag"].setText(f"{total_magnification:.1f}")
            self.result_labels["cam_eff_pixel"].setText(f"{effective_pixel:.0f}")
            self.result_labels["cam_fov"].setText(f"{fov_v:.0f}×{fov_h:.0f}")
            self.result_labels["cam_distance_user"].setText(f"{camera_distance}")

            self.result_labels["cond_type"].setText(self.condenser_combo.currentText())
            self.result_labels["cond_source"].setText(f"{source_distance / 1000:.1f}")  # Display in meters
            self.result_labels["cond_inner"].setText(f"{cond_config['inner_dia']}")
            self.result_labels["cond_outer"].setText(f"{cond_config['outer_dia']}")
            self.result_labels["cond_length"].setText(f"{cond_config['length']}")
            self.result_labels["cond_focal"].setText(f"{cond_config['focal_length']}")
            self.result_labels["cond_position"].setText(f"{cond_config['focal_length']}")

            # Update schematic with all optical parameters
            self.schematic.update_parameters(
                source_to_cond=source_distance,  # User-editable source distance in mm
                cond_to_sample=0,  # Not calculated
                sample_to_zp=focal_length,
                zp_to_cam=camera_distance,
                cond_name=self.condenser_combo.currentText(),
                zp_name=self.zp_combo.currentText(),
                cam_name=self.camera_combo.currentText(),
                zp_diameter=diameter,  # Zone plate diameter in μm
                zp_na=na / 1000,  # Convert mrad to rad for scaling
                cond_inner=cond_config['inner_dia'],  # Condenser inner diameter in μm
                cond_outer=cond_config['outer_dia']  # Condenser outer diameter in μm
            )

            self.status_label.setText(f"✓ Calculated for {energy} eV - λ = {wavelength:.3f} nm - Binning: {binning}×")

        except Exception as e:
            self.status_label.setText(f"❌ Calculation error: {str(e)}")

    def reset(self):
        """Reset to default values"""
        self.energy_edit.setText("8000")
        self.zp_combo.setCurrentText("30nm ZP")
        self.condenser_combo.setCurrentText("Sigray")
        self.camera_combo.setCurrentText("ORX-10G-310S9M")
        self.optical_mag_combo.setCurrentText("2x Objective")
        self.calculate()
        self.status_label.setText("🔄 Reset to default values")

    def save_config_with_feedback(self):
        """Save config with user feedback"""
        if self.save_config():
            QMessageBox.information(self, "Save Complete", "✓ Configuration saved successfully!")

    def export_config(self):
        """Export configuration to file"""
        filename, _ = QFileDialog.getSaveFileName(
            self, "Export Configuration", "", "JSON files (*.json)")

        if filename:
            try:
                with open(filename, 'w') as f:
                    json.dump(self.configs, f, indent=2)
                QMessageBox.information(self, "Export Complete",
                                      f"✓ Configuration exported to {filename}")
            except Exception as e:
                QMessageBox.critical(self, "Export Error", f"❌ Failed to export: {e}")

    def import_config(self):
        """Import configuration from file"""
        filename, _ = QFileDialog.getOpenFileName(
            self, "Import Configuration", "", "JSON files (*.json)")

        if filename:
            try:
                with open(filename, 'r') as f:
                    imported_config = json.load(f)
                self.configs = self.merge_configs(self.default_configs, imported_config)
                self.update_combo_boxes()
                self.calculate()
                QMessageBox.information(self, "Import Complete",
                                      f"✓ Configuration imported from {filename}")
                self.status_label.setText("✓ Configuration imported successfully")
            except Exception as e:
                QMessageBox.critical(self, "Import Error", f"❌ Failed to import: {e}")

    def reset_to_defaults(self):
        """Reset all configurations to defaults"""
        reply = QMessageBox.question(
            self, "Reset Configuration",
            "Are you sure you want to reset all configurations to defaults?",
            QMessageBox.Yes | QMessageBox.No)

        if reply == QMessageBox.Yes:
            self.configs = self.default_configs.copy()
            self.update_combo_boxes()
            self.save_config()
            self.calculate()
            QMessageBox.information(self, "Reset Complete", "✓ Configuration reset to defaults")
            self.status_label.setText("✓ Configuration reset to defaults")

    def export_results(self):
        """Export calculation results to file"""
        try:
            energy = self.energy_edit.text()
            wavelength = self.wavelength_label.text()

            results = f"""X-ray Optics Calculation Results
================================
Timestamp: {datetime.now().strftime("%Y-%m-%d %H:%M:%S")}
Configuration: {self.config_file}

Input Parameters:
- Energy: {energy} eV
- Wavelength: {wavelength} nm
- Zone Plate: {self.zp_combo.currentText()}
- Condenser: {self.condenser_combo.currentText()}
- Camera: {self.camera_combo.currentText()}
- Optical Magnification: {self.optical_mag_combo.currentText()}

User Parameters:
- Camera Distance: {self.camera_distance_edit.text()} mm
- Binning: {self.binning_edit.text()}×
- Exposure Time: {self.exposure_time_edit.text()} s
- Sample-Detector Distance: {self.sample_det_distance_edit.text()} mm
- Beam Current: {self.beam_current_edit.text()} mA

Zone Plate Results:
- Diameter: {self.result_labels["zp_diameter"].text()} μm
- Outer Ring (Δrn): {self.result_labels["zp_drn"].text()} nm
- Smallest Feature: {self.result_labels["zp_smallest"].text()} nm
- Focal Length: {self.result_labels["zp_focal"].text()} mm
- Numerical Aperture: {self.result_labels["zp_na"].text()} mrad
- Number of Zones: {self.result_labels["zp_zones"].text()}
- Motor Position: {self.result_labels["zp_position"].text()} mm

Camera & Imaging Results:
- Pixel Size: {self.result_labels["cam_pixel"].text()} μm
- Resolution: {self.result_labels["cam_pixels"].text()} px
- Effective Resolution: {self.result_labels["cam_eff_pixels"].text()} px
- Optical Magnification: {self.result_labels["opt_mag"].text()}×
- Total Magnification: {self.result_labels["total_mag"].text()}×
- Effective Pixel Size: {self.result_labels["cam_eff_pixel"].text()} nm
- Field of View: {self.result_labels["cam_fov"].text()} μm
- Camera Distance: {self.result_labels["cam_distance_user"].text()} mm

Condenser Results:
- Type: {self.result_labels["cond_type"].text()}
- Source Distance: {self.result_labels["cond_source"].text()} m
- Inner Diameter: {self.result_labels["cond_inner"].text()} μm
- Outer Diameter: {self.result_labels["cond_outer"].text()} μm
- Length: {self.result_labels["cond_length"].text()} mm
- Focal Length: {self.result_labels["cond_focal"].text()} mm
- Motor Position: {self.result_labels["cond_position"].text()} mm
"""

            filename = f"optics_results_{datetime.now().strftime('%Y%m%d_%H%M%S')}.txt"
            with open(filename, "w", encoding='utf-8') as f:
                f.write(results)

            QMessageBox.information(self, "Export Complete", f"✓ Results saved to {filename}")
            self.status_label.setText(f"✓ Results exported to {filename}")

        except Exception as e:
            QMessageBox.critical(self, "Export Error", f"❌ Failed to export: {str(e)}")

    def apply_styling(self):
        """Apply pystream-style dark theme"""
        self.setStyleSheet("""
            QMainWindow, QWidget {
                background-color: #1a1a1a;
                color: #e0e0e0;
            }
            QPushButton {
                background-color: #2d2d2d;
                color: #e0e0e0;
                padding: 6px 12px;
                border: 1px solid #404040;
                border-radius: 3px;
            }
            QPushButton:hover {
                background-color: #3a3a3a;
                border: 1px solid #505050;
            }
            QPushButton:pressed {
                background-color: #252525;
            }
            QPushButton:checked {
                background-color: #1e5a8e;
                border: 1px solid #2980b9;
            }
            QLineEdit {
                background-color: #2d2d2d;
                color: #e0e0e0;
                padding: 4px 8px;
                border: 1px solid #404040;
                border-radius: 3px;
            }
            QLineEdit:focus {
                border: 1px solid #2980b9;
            }
            QCheckBox {
                color: #e0e0e0;
                spacing: 5px;
            }
            QCheckBox::indicator {
                width: 16px;
                height: 16px;
                border: 1px solid #404040;
                border-radius: 3px;
                background-color: #2d2d2d;
            }
            QCheckBox::indicator:checked {
                background-color: #2980b9;
                border: 1px solid #3a95d8;
            }
            QComboBox {
                background-color: #2d2d2d;
                color: #e0e0e0;
                padding: 4px 8px;
                border: 1px solid #404040;
                border-radius: 3px;
            }
            QComboBox:focus {
                border: 1px solid #2980b9;
            }
            QComboBox::drop-down {
                border: none;
                width: 20px;
            }
            QComboBox::down-arrow {
                image: none;
                border-left: 5px solid transparent;
                border-right: 5px solid transparent;
                border-top: 5px solid #e0e0e0;
                margin-right: 5px;
            }
            QComboBox QAbstractItemView {
                background-color: #2d2d2d;
                color: #e0e0e0;
                selection-background-color: #2980b9;
                selection-color: white;
                border: 1px solid #404040;
            }
            QGroupBox {
                color: #e0e0e0;
                border: 1px solid #404040;
                border-radius: 5px;
                margin-top: 12px;
                padding-top: 12px;
                font-weight: bold;
            }
            QGroupBox::title {
                subcontrol-origin: margin;
                left: 10px;
                padding: 0 5px;
            }
            QLabel {
                color: #e0e0e0;
            }
            QTabWidget::pane {
                border: 1px solid #404040;
                border-radius: 4px;
                background-color: #1a1a1a;
                top: -1px;
            }
            QTabBar::tab {
                background-color: #2d2d2d;
                color: #e0e0e0;
                padding: 8px 20px;
                margin-right: 2px;
                border-top-left-radius: 4px;
                border-top-right-radius: 4px;
                min-width: 100px;
                border: 1px solid #404040;
            }
            QTabBar::tab:selected {
                background-color: #1a1a1a;
                border-bottom: 2px solid #2980b9;
            }
            QTabBar::tab:hover {
                background-color: #3a3a3a;
            }
            QTableWidget {
                background-color: #2d2d2d;
                color: #e0e0e0;
                gridline-color: #404040;
                border: 1px solid #404040;
                border-radius: 4px;
            }
            QTableWidget::item {
                padding: 5px;
            }
            QTableWidget::item:selected {
                background-color: #2980b9;
                color: white;
            }
            QHeaderView::section {
                background-color: #2d2d2d;
                color: #e0e0e0;
                padding: 5px;
                border: 1px solid #404040;
                font-weight: bold;
            }
            QDialog {
                background-color: #1a1a1a;
                color: #e0e0e0;
            }
            QDialogButtonBox QPushButton {
                min-width: 80px;
            }
            QLabel#statusLabel {
                background-color: #2d2d2d;
                color: #e0e0e0;
                padding: 8px;
                border-top: 1px solid #404040;
                border-radius: 0px;
            }
            QFrame[frameShape="4"], QFrame[frameShape="5"] {
                color: #404040;
            }
        """)


def main():
    app = QApplication(sys.argv)
    window = ModernOpticsCalculator()
    window.show()
    sys.exit(app.exec_())


if __name__ == "__main__":
    main()
