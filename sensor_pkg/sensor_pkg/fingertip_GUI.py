import sys
import numpy as np
import time
import serial
import threading
import csv
from datetime import datetime
from PyQt5.QtWidgets import (
    QApplication, QMainWindow, QWidget, QVBoxLayout, QLabel, QHBoxLayout,
    QPushButton, QFileDialog, QMessageBox
)
from PyQt5.QtCore import QTimer
import pyqtgraph.opengl as gl
import pyqtgraph as pg


TAXEL_GRID = (2, 2)
TAXEL_SPACING = 2.0
SEMISPHERE_RADIUS = 0.7
ARROW_MAX_FORCE = 1000
FORCE_THRESHOLD = 180#500
MIN_ARROW_LENGTH = SEMISPHERE_RADIUS * 1.05
MAX_ARROW_LENGTH = 1.4

single_contact = False

force_data = np.zeros((4, 3))

def parse_sensor_line(line):
    try:
        parts = line.split()
        if len(parts) != 12:
            return None
        values = list(map(float, parts))
        forces = np.array(values).reshape((4,3))
        forces[(forces >= -FORCE_THRESHOLD) & (forces <= FORCE_THRESHOLD)] = 0
        return forces
    except Exception:
        return None

class SerialReaderThread(threading.Thread):
    def __init__(self, port='/dev/ttyACM0', baud=115200):
        super().__init__()
        self.port = port
        self.baud = baud
        self.running = True
        self.ser = None

    def run(self):
        global force_data
        last_time = time.time()
        count = 0
        try:
            self.ser = serial.Serial(self.port, self.baud, timeout=1)
            time.sleep(2)
            print("start reading...\n")
            while self.running:
                if self.ser.in_waiting > 0:
                    line = self.ser.readline().decode('utf-8', errors='ignore').strip()
                    forces = parse_sensor_line(line)
                    if forces is not None:
                        force_data = forces
                        count += 1
                        # Check frequency every second
                        current_time = time.time()
                        if current_time - last_time >= 1.0:
                            frequency = count / (current_time - last_time)
                            print(f"[{self.port}] Reading frequency: {frequency:.1f} Hz")
                            count = 0
                            last_time = current_time
        except Exception as e:
            print(f"Serial thread error: {e}")
        finally:
            if self.ser:
                self.ser.close()

def read_sensor_data():
    global force_data
    return force_data

def semisphere_mesh(radius=SEMISPHERE_RADIUS, rows=20, cols=40):
    phi = np.linspace(0, np.pi/2, rows)
    theta = np.linspace(0, 2*np.pi, cols)
    phi, theta = np.meshgrid(phi, theta)
    x = radius * np.sin(phi) * np.cos(theta)
    y = radius * np.sin(phi) * np.sin(theta)
    z = radius * np.cos(phi)
    points = np.vstack([x.flatten(), y.flatten(), z.flatten()]).T
    faces = []
    for i in range(cols-1):
        for j in range(rows-1):
            idx = i*rows + j
            faces.append([idx, idx+1, idx+rows])
            faces.append([idx+1, idx+rows+1, idx+rows])
    faces = np.array(faces)
    return gl.MeshData(vertexes=points, faces=faces)

class TaxelVisualizer(QMainWindow):
    def __init__(self):
        super().__init__()
        self.setWindowTitle('2x2 Taxel Sensor Visualization')
        self.resize(1400, 1200)
        print("started")

        central_widget = QWidget()
        self.setCentralWidget(central_widget)
        self.main_layout = QVBoxLayout(central_widget)

        self.view = gl.GLViewWidget()
        self.view.setBackgroundColor(220, 220, 220)
        self.view.setCameraPosition(distance=8, elevation=30, azimuth=30)
        self.main_layout.addWidget(self.view, stretch=2)

        grid = gl.GLGridItem()
        grid.setSize(5, 5)
        grid.setSpacing(0.5, 0.5)
        self.view.addItem(grid)

        self.radius = SEMISPHERE_RADIUS
        self.arrow_min_length = MIN_ARROW_LENGTH
        self.arrow_max_length = MAX_ARROW_LENGTH

        self.display_order = [0, 1, 3, 2]
        self.grid_coords = [
            ((0 - 0.5) * TAXEL_SPACING, (0 - 0.5) * TAXEL_SPACING),  # Taxel 1
            ((1 - 0.5) * TAXEL_SPACING, (0 - 0.5) * TAXEL_SPACING),  # Taxel 2
            ((0 - 0.5) * TAXEL_SPACING, (1 - 0.5) * TAXEL_SPACING),  # Taxel 3
            ((1 - 0.5) * TAXEL_SPACING, (1 - 0.5) * TAXEL_SPACING),  # Taxel 4
        ]

        self.taxel_objs = []
        self.force_labels = []

        # --- Label positions (over the 3D scene, above the time plots) ---
        scene_w, scene_h = 1400, 800
        label_w, label_h = 120, 60
        self.label_positions = [
            (scene_w//2 - label_w//2, 50),                # Taxel 1: Top center
            (scene_w//2 - 480, scene_h//2 - label_h//2),  # Taxel 2: Left center (more left)
            (scene_w//2 - label_w//2, scene_h - 160),     # Taxel 4: Bottom center (a bit higher)
            (scene_w//2 + 250, scene_h//2 - label_h//2),  # Taxel 3: Right center (more central)
        ]

        for idx, (x, y) in enumerate(self.grid_coords):
            mesh = gl.GLMeshItem(
                meshdata=semisphere_mesh(radius=self.radius),
                smooth=True, color=(1, 1, 1, 1), shader='balloon', drawEdges=False, glOptions='translucent')
            mesh.translate(x, y, 0)
            self.view.addItem(mesh)

            cyl_mesh = gl.MeshData.cylinder(rows=10, cols=20, radius=[0.08, 0.08], length=self.arrow_min_length)
            arrow_cyl = gl.GLMeshItem(meshdata=cyl_mesh, smooth=True, color=(0, 0, 1, 1), shader='shaded', drawEdges=False)
            cone_mesh = gl.MeshData.cylinder(rows=10, cols=20, radius=[0.16, 0], length=0.25)
            arrow_cone = gl.GLMeshItem(meshdata=cone_mesh, smooth=True, color=(0, 0, 1, 1), shader='shaded', drawEdges=False)
            arrow_cone.translate(0, 0, self.arrow_min_length)
            self.view.addItem(arrow_cyl)
            self.view.addItem(arrow_cone)

            base_disk_mesh = gl.MeshData.cylinder(rows=10, cols=20, radius=[0.08, 0.08], length=0.01)
            base_disk_item = gl.GLMeshItem(meshdata=base_disk_mesh, smooth=True, color=(0, 0, 1, 1), shader='balloon', drawEdges=False)
            tip_disk_mesh = gl.MeshData.cylinder(rows=10, cols=20, radius=[0.16, 0.16], length=0.01)
            tip_disk_item = gl.GLMeshItem(meshdata=tip_disk_mesh, smooth=True, color=(0, 0, 1, 1), shader='balloon', drawEdges=False)
            tip_disk_item.translate(0, 0, self.arrow_min_length)
            self.view.addItem(base_disk_item)
            self.view.addItem(tip_disk_item)

            tip_marker = gl.GLMeshItem(
                meshdata=gl.MeshData.cylinder(rows=10, cols=40, radius=[0.12, 0.12], length=0.08),
                smooth=True, color=(0.2, 0.2, 0.2, 1), shader='balloon', drawEdges=False)
            self.view.addItem(tip_marker)

            label = QLabel(self)
            label.setStyleSheet("background: rgba(255,255,255,0.8); color: black; font-size: 14px; border: 1px solid gray;")
            label.setFixedWidth(label_w)
            label.setFixedHeight(label_h)
            label.move(*self.label_positions[idx])
            label.show()
            self.force_labels.append(label)

            self.taxel_objs.append({
                'mesh': mesh,
                'arrow_cyl': arrow_cyl,
                'arrow_cone': arrow_cone,
                'base_disk': base_disk_item,
                'tip_disk': tip_disk_item,
                'tip_marker': tip_marker,
                'center': (x, y, 0)
            })
            


        # --- Plots for each taxel ---
        self.plot_fx = []
        self.plot_fy = []
        self.plot_fz = []
        self.fx_data = [ [] for _ in range(4)]
        self.fy_data = [ [] for _ in range(4)]
        self.fz_data = [ [] for _ in range(4)]
        self.max_points = 300

        plot_layout = QHBoxLayout()
        for idx in range(4):
            pw = pg.PlotWidget(title=f"Taxel {idx+1}")
            pw.setLabel('left', 'Force (N)')
            pw.setLabel('bottom', 'Samples')
            pw.addLegend()
            pw.showGrid(x=True, y=True)
            fx_curve = pw.plot(pen='b', name='Fx')
            fy_curve = pw.plot(pen='g', name='Fy')
            fz_curve = pw.plot(pen='r', name='Fz')
            self.plot_fx.append(fx_curve)
            self.plot_fy.append(fy_curve)
            self.plot_fz.append(fz_curve)
            plot_layout.addWidget(pw)
        self.main_layout.addLayout(plot_layout, stretch=1)

        # ---- Buttons ----
        btn_layout = QHBoxLayout()
        self.record_btn = QPushButton("Start Recording")
        self.record_btn.clicked.connect(self.toggle_recording)
        btn_layout.addWidget(self.record_btn)

        self.save_btn = QPushButton("Save Data")
        self.save_btn.clicked.connect(self.save_data)
        self.save_btn.setEnabled(False)
        btn_layout.addWidget(self.save_btn)

        self.video_btn = QPushButton("Record Video")
        self.video_btn.clicked.connect(self.toggle_video_recording)
        btn_layout.addWidget(self.video_btn)

        btn_layout.addStretch()
        self.main_layout.addLayout(btn_layout)

        # Recording state
        self.recording = False
        self.recorded_data = []
        self.video_recording = False
        self.frames = []

        self.timer = QTimer()
        self.timer.timeout.connect(self.update_arrows)
        # self.timer.start(30)

    # ADD THIS METHOD
    def showEvent(self, event):
        """
        This event is called by Qt right before the widget is shown.
        It's the perfect place to start timers that update the GL scene.
        """
        super().showEvent(event) # Call the parent class's implementation
        if not self.timer.isActive():
            self.timer.start(30)
        

    def update_arrows(self):
        forces = read_sensor_data()
        k_deform = 0.03

        if single_contact:
            mags = [np.linalg.norm(f) for f in forces]
            idx_max = int(np.argmax(mags))
            filtered_forces = np.zeros_like(forces)
            filtered_forces[idx_max] = forces[idx_max]
            forces = filtered_forces

        for idx, obj in enumerate(self.taxel_objs):
            sensor_idx = self.display_order[idx]
            fx, fy, fz = forces[sensor_idx]

            mag_xy = np.linalg.norm([fx, fy])
            abs_fz = abs(fz)
            if mag_xy > 1e-6 and abs_fz > 0:
                tilt_axis = np.array([-fy, fx, 0])
                tilt_axis_norm = np.linalg.norm(tilt_axis)
                if tilt_axis_norm > 1e-6:
                    tilt_axis = tilt_axis / tilt_axis_norm
                else:
                    tilt_axis = np.array([0,0,1])
                tilt_angle = np.degrees(np.arctan2(mag_xy, abs_fz))
            else:
                tilt_axis = np.array([0,0,1])
                tilt_angle = 0

            # Use abs(fz) for arrow length
            length = self.arrow_min_length + (abs_fz / ARROW_MAX_FORCE) * (self.arrow_max_length - self.arrow_min_length)

            x0, y0, z0 = obj['center']

            r = np.clip(abs(fz) / ARROW_MAX_FORCE, 0, 1)
            g = np.clip(abs(fy) / ARROW_MAX_FORCE, 0, 1)
            b = np.clip(abs(fx) / ARROW_MAX_FORCE, 0, 1)
            if r == 0 and g == 0 and b == 0:
                color = (1, 1, 1, 1)
            else:
                color = (b, g, r, 0.25)
            obj['mesh'].setColor(color)

            obj['mesh'].resetTransform()
            scale_z = 1 - k_deform * np.clip(abs(fz), 0, ARROW_MAX_FORCE)
            scale_z = max(0.5, scale_z)
            obj['mesh'].translate(x0, y0, z0)
            obj['mesh'].scale(1, 1, scale_z)

            obj['arrow_cyl'].resetTransform()
            obj['arrow_cone'].resetTransform()
            obj['arrow_cyl'].scale(1, 1, length/self.arrow_min_length)
            obj['arrow_cone'].resetTransform()
            obj['arrow_cone'].translate(0, 0, length)
            if tilt_angle != 0:
                obj['arrow_cyl'].rotate(tilt_angle, *tilt_axis)
                obj['arrow_cone'].rotate(tilt_angle, *tilt_axis)
            obj['arrow_cyl'].translate(x0, y0, z0)
            obj['arrow_cone'].translate(x0, y0, z0)
            fz_norm = np.clip((fz + ARROW_MAX_FORCE) / (2 * ARROW_MAX_FORCE), 0, 1)
            arrow_color = (fz_norm, 0, 1-fz_norm, 1)
            obj['arrow_cyl'].setColor(arrow_color)
            obj['arrow_cone'].setColor(arrow_color)

            obj['base_disk'].resetTransform()
            if tilt_angle != 0:
                obj['base_disk'].rotate(tilt_angle, *tilt_axis)
            obj['base_disk'].translate(x0, y0, z0)
            obj['base_disk'].setColor(arrow_color)
            obj['tip_disk'].resetTransform()
            obj['tip_disk'].translate(0, 0, length)
            if tilt_angle != 0:
                obj['tip_disk'].rotate(tilt_angle, *tilt_axis)
            obj['tip_disk'].translate(x0, y0, z0)
            obj['tip_disk'].setColor(arrow_color)

            obj['tip_marker'].resetTransform()
            tip_z = z0 + self.radius * scale_z
            if tilt_angle != 0:
                obj['tip_marker'].rotate(tilt_angle, *tilt_axis)
            obj['tip_marker'].translate(x0, y0, tip_z)

            # --- Swap the printed values for F3 and F4 in the labels ---
            label = self.force_labels[idx]
            if idx == 2:  # bottom center: show F4
                fidx = self.display_order[3]
                label.setText(f"Fx4: {forces[fidx][0]:.0f}\nFy4: {forces[fidx][1]:.0f}\nFz4: {forces[fidx][2]:.0f}")
            elif idx == 3:  # right center: show F3
                fidx = self.display_order[2]
                label.setText(f"Fx3: {forces[fidx][0]:.0f}\nFy3: {forces[fidx][1]:.0f}\nFz3: {forces[fidx][2]:.0f}")
            else:  # top center, left center: show F1, F2 as normal
                label.setText(f"Fx{self.display_order[idx]+1}: {fx:.0f}\nFy{self.display_order[idx]+1}: {fy:.0f}\nFz{self.display_order[idx]+1}: {fz:.0f}")

            self.fx_data[idx].append(fx)
            self.fy_data[idx].append(fy)
            self.fz_data[idx].append(fz)
            if len(self.fx_data[idx]) > self.max_points:
                self.fx_data[idx] = self.fx_data[idx][-self.max_points:]
                self.fy_data[idx] = self.fy_data[idx][-self.max_points:]
                self.fz_data[idx] = self.fz_data[idx][-self.max_points:]
            self.plot_fx[idx].setData(self.fx_data[idx])
            self.plot_fy[idx].setData(self.fy_data[idx])
            self.plot_fz[idx].setData(self.fz_data[idx])

        if self.recording:
            timestamp = datetime.now().isoformat()
            for i in range(4):
                fx, fy, fz = forces[self.display_order[i]]
                self.recorded_data.append([timestamp, i+1, fx, fy, fz])

        if self.video_recording:
            img = self.view.renderToArray((800, 600))
            self.frames.append(img)

    def toggle_recording(self):
        if not getattr(self, 'recording', False):
            self.recorded_data = []
            self.recording = True
            self.record_btn.setText("Stop Recording")
            self.save_btn.setEnabled(False)
        else:
            self.recording = False
            self.record_btn.setText("Start Recording")
            self.save_btn.setEnabled(True)
            QMessageBox.information(self, "Recording stopped", "Data recording stopped. Click 'Save Data' to save.")

    def save_data(self):
        if not hasattr(self, 'recorded_data') or not self.recorded_data:
            QMessageBox.warning(self, "No data", "No recorded data to save.")
            return
        options = QFileDialog.Options()
        filename, _ = QFileDialog.getSaveFileName(self, "Save Data", "taxel_data.csv", "CSV Files (*.csv)", options=options)
        if filename:
            with open(filename, 'w', newline='') as f:
                writer = csv.writer(f)
                writer.writerow(['timestamp', 'taxel', 'Fx', 'Fy', 'Fz'])
                writer.writerows(self.recorded_data)
            QMessageBox.information(self, "Saved", f"Data saved to {filename}")

    def toggle_video_recording(self):
        if not getattr(self, 'video_recording', False):
            self.frames = []
            self.video_recording = True
            self.video_btn.setText("Stop Video")
        else:
            self.video_recording = False
            self.video_btn.setText("Record Video")
            if not self.frames:
                QMessageBox.warning(self, "No frames", "No video frames captured.")
                return
            try:
                import imageio
                options = QFileDialog.Options()
                filename, _ = QFileDialog.getSaveFileName(self, "Save Video", "taxel_scene.mp4", "MP4 Files (*.mp4)", options=options)
                if filename:
                    frames_uint8 = [np.clip((frame * 255), 0, 255).astype(np.uint8) for frame in self.frames]
                    imageio.mimsave(filename, frames_uint8, fps=30)
                    QMessageBox.information(self, "Saved", f"Video saved to {filename}")
            except ImportError:
                QMessageBox.warning(self, "Missing package", "Install 'imageio' and 'imageio-ffmpeg' to enable video export:\npip install imageio imageio-ffmpeg")

if __name__ == '__main__':
    serial_thread = SerialReaderThread(port='/dev/ttyACM0', baud=115200)
    serial_thread.daemon = True
    serial_thread.start()

    app = QApplication(sys.argv)
    window = TaxelVisualizer()
    window.show()
    sys.exit(app.exec_())
