import sys
import time
import threading
import numpy as np
import csv
from datetime import datetime
from PyQt5.QtWidgets import QApplication, QMainWindow, QWidget, QVBoxLayout, QPushButton, QLabel, QFileDialog, QMessageBox, QHBoxLayout
from PyQt5.QtCore import QTimer
import pyqtgraph.opengl as gl
import pyqtgraph as pg
import serial
import minimalmodbus as mm
import libscrc

# ========== CONFIGURABLE PARAMETERS ==========
MAX_FORCE = 15  # Clamp force values to +/- MAX_FORCE for visualization

# ========== SENSOR DATA SHARED STATE ==========
force_data = {'x': 0, 'y': 0, 'z': 0, 't1': 0, 't2': 0, 't3': 0, 'freq': 0}

# ========== SENSOR UTILITIES ==========
def forceFromSerialMessage(serialMessage, zeroRef=[0,0,0,0,0,0]):
    forceTorque = [0,0,0,0,0,0]
    forceTorque[0] = round(int.from_bytes(serialMessage[2:4], byteorder='little', signed=True)/100 - zeroRef[0], 2)
    forceTorque[1] = round(int.from_bytes(serialMessage[4:6], byteorder='little', signed=True)/100 - zeroRef[1], 2)
    forceTorque[2] = round(int.from_bytes(serialMessage[6:8], byteorder='little', signed=True)/100 - zeroRef[2], 2)
    forceTorque[3] = round(int.from_bytes(serialMessage[8:10], byteorder='little', signed=True)/1000 - zeroRef[3], 2)
    forceTorque[4] = round(int.from_bytes(serialMessage[10:12], byteorder='little', signed=True)/1000 - zeroRef[4], 2)
    forceTorque[5] = round(int.from_bytes(serialMessage[12:14], byteorder='little', signed=True)/1000 - zeroRef[5], 2)
    return forceTorque

def crcCheck(serialMessage):
    crc = int.from_bytes(serialMessage[14:16], byteorder='little', signed=False)
    crcCalc = libscrc.modbus(serialMessage[0:14])
    return crc == crcCalc

def clamp_force_values(fx, fy, fz, max_force=MAX_FORCE):
    fx_clamped = np.clip(fx, -max_force, max_force)
    fy_clamped = np.clip(fy, -max_force, max_force)
    fz_clamped = np.clip(fz, -max_force, max_force)
    return fx_clamped, fy_clamped, fz_clamped

# ========== SENSOR THREAD ==========
class SensorThread(threading.Thread):
    def __init__(self, port='/dev/ttyUSB0', baudrate=19200):
        super().__init__()
        self.port = port
        self.baudrate = baudrate
        self.running = True

    def run(self):
        try:
            # Deactivate streaming mode
            ser = serial.Serial(port=self.port, baudrate=self.baudrate, bytesize=8, parity='N', stopbits=1, timeout=1)
            packet = bytearray([0xff]*50)
            ser.write(packet)
            ser.close()

            # Activate streaming mode
            mm.BAUDRATE = self.baudrate
            mm.BYTESIZE = 8
            mm.PARITY = 'N'
            mm.STOPBITS = 1
            mm.TIMEOUT = 1
            ft300 = mm.Instrument(self.port, slaveaddress=9)
            ft300.close_port_after_each_call = True
            ft300.write_register(410, 0x0200)
            del ft300

            # Open serial connection
            ser = serial.Serial(port=self.port, baudrate=self.baudrate, bytesize=8, parity='N', stopbits=1, timeout=1)
            startTime = time.time()

            # Initialize stream reading
            STARTBYTES = bytes([0x20, 0x4e])
            ser.read_until(STARTBYTES)
            data = ser.read_until(STARTBYTES)
            dataArray = bytearray(data)
            dataArray = STARTBYTES + dataArray[:-2]
            if not crcCheck(dataArray):
                print("CRC ERROR: Serial message and the CRC does not match")
                return
            zeroRef = forceFromSerialMessage(dataArray)

            nbrMessages = 0
            while self.running:
                data = ser.read_until(STARTBYTES)
                dataArray = bytearray(data)
                dataArray = STARTBYTES + dataArray[:-2]
                if not crcCheck(dataArray):
                    continue
                forceTorque = forceFromSerialMessage(dataArray, zeroRef)
                nbrMessages += 1
                elapsedTime = time.time() - startTime
                freq = round(nbrMessages / elapsedTime) if elapsedTime > 0 else 0
                # Update shared state
                force_data['x'] = forceTorque[0]
                force_data['y'] = forceTorque[1]
                force_data['z'] = forceTorque[2]
                force_data['t1'] = forceTorque[3]
                force_data['t2'] = forceTorque[4]
                force_data['t3'] = forceTorque[5]
                force_data['freq'] = freq
        except Exception as e:
            print(f"Sensor thread error: {e}")

# ========== MAIN GUI ==========
class Sensor3DVisualizer(QMainWindow):
    def __init__(self):
        super().__init__()
        self.setWindowTitle('FT300 Sensor 3D Visualization')
        self.resize(1000, 1000)
        self.recording = False
        self.recorded_data = []
        self.video_recording = False

        # ---- Central Widget Layout ----
        central_widget = QWidget()
        self.setCentralWidget(central_widget)
        layout = QVBoxLayout(central_widget)

        # ---- 3D View ----
        self.view = gl.GLViewWidget()
        self.view.setBackgroundColor(220, 220, 220)
        self.view.setCameraPosition(distance=5, elevation=20, azimuth=30)
        layout.addWidget(self.view, stretch=2)

        # ---- 3D Grid ----
        grid = gl.GLGridItem()
        grid.setSize(4, 4)
        grid.setSpacing(0.2, 0.2)
        self.view.addItem(grid)

        # ---- Pin arrow and base cylinder ----
        self.pin_length = 1.5
        self.pin_cylinder = gl.GLMeshItem(
            meshdata=gl.MeshData.cylinder(rows=10, cols=20, radius=[0.07, 0.07], length=self.pin_length),
            smooth=True, color=(0, 0, 1, 1), shader='shaded', drawEdges=False)
        self.pin_cone = gl.GLMeshItem(
            meshdata=gl.MeshData.cylinder(rows=10, cols=20, radius=[0.14, 0], length=0.3),
            smooth=True, color=(0, 0, 1, 1), shader='shaded', drawEdges=False)
        self.pin_cone.translate(0, 0, self.pin_length)
        self.view.addItem(self.pin_cylinder)
        self.view.addItem(self.pin_cone)

        self.base_cylinder = gl.GLMeshItem(
            meshdata=gl.MeshData.cylinder(rows=10, cols=20, radius=[0.3, 0.3], length=0.5),
            smooth=True, color=(0.5, 0.5, 0.5, 1), shader='shaded', drawEdges=False)
        self.base_cylinder.translate(0, 0, -0.25)
        self.view.addItem(self.base_cylinder)

        # ---- Frequency label ----
        freq_layout = QHBoxLayout()
        self.freq_label = QLabel("Frequency: 0 Hz")
        self.freq_label.setStyleSheet("font-size: 16px; font-weight: bold;")
        freq_layout.addWidget(self.freq_label)
        freq_layout.addStretch()
        layout.addLayout(freq_layout)

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
        layout.addLayout(btn_layout)

        # ---- Real-time PlotWidget for Fx, Fy, Fz ----
        self.plot_widget = pg.PlotWidget(title="Force vs Time")
        self.plot_widget.setLabel('left', 'Force (N)')
        self.plot_widget.setLabel('bottom', 'Samples')
        self.plot_widget.addLegend()
        self.plot_widget.showGrid(x=True, y=True)
        layout.addWidget(self.plot_widget, stretch=1)

        self.plot_fx = self.plot_widget.plot(pen='r', name='Fx')
        self.plot_fy = self.plot_widget.plot(pen='g', name='Fy')
        self.plot_fz = self.plot_widget.plot(pen='b', name='Fz')
        self.fx_data = []
        self.fy_data = []
        self.fz_data = []
        self.max_points = 300  # Show last 300 samples

        # ---- Timer for updates ----
        self.timer = QTimer()
        self.timer.timeout.connect(self.read_and_update)
        self.timer.start(30)  # ~33Hz update

        self.frames = []
        self.video_start_time = None

    def read_and_update(self):
        fx = force_data['x']
        fy = force_data['y']
        fz = force_data['z']
        t1 = force_data['t1']
        t2 = force_data['t2']
        t3 = force_data['t3']
        freq = force_data['freq']

        fx_clamped, fy_clamped, fz_clamped = clamp_force_values(fx, fy, fz, max_force=MAX_FORCE)
        fx_plot = int(fx_clamped)
        fy_plot = int(fy_clamped)
        fz_plot = int(fz_clamped)

        self.update_visualization(fx_plot, fy_plot, fz_plot, t1, t2, t3)
        self.freq_label.setText(f"Frequency: {freq} Hz")

        # --- Update plot data ---
        self.fx_data.append(fx_plot)
        self.fy_data.append(fy_plot)
        self.fz_data.append(fz_plot)
        if len(self.fx_data) > self.max_points:
            self.fx_data = self.fx_data[-self.max_points:]
            self.fy_data = self.fy_data[-self.max_points:]
            self.fz_data = self.fz_data[-self.max_points:]
        self.plot_fx.setData(self.fx_data)
        self.plot_fy.setData(self.fy_data)
        self.plot_fz.setData(self.fz_data)

        # If recording, save data (with full precision)
        if self.recording:
            now = datetime.now().isoformat()
            self.recorded_data.append([now, fx, fy, fz, t1, t2, t3, freq])

        # If video recording, grab frame
        if self.video_recording:
            img = self.view.renderToArray((800, 600))
            self.frames.append(img)

    def update_visualization(self, fx, fy, fz, t1, t2, t3):
        # Pin arrow color: blue (low fz) to red (high fz)
        fz_norm = np.clip((fz + MAX_FORCE) / (2 * MAX_FORCE), 0, 1)
        color = (fz_norm, 0, 1 - fz_norm, 1)
        self.pin_cylinder.setColor(color)
        self.pin_cone.setColor(color)

        # Pin arrow tilt and length
        self.pin_cylinder.resetTransform()
        self.pin_cone.resetTransform()

        pin_length = 1.0 + np.clip(abs(fz) / MAX_FORCE, 0, 1)  # Between 1 and 2 units
        self.pin_cylinder.scale(1, 1, pin_length / self.pin_length)
        self.pin_cone.translate(0, 0, pin_length)

        max_tilt_deg = 20
        fx_norm = np.clip(fx / MAX_FORCE, -1, 1)
        fy_norm = np.clip(fy / MAX_FORCE, -1, 1)
        angle_x = fy_norm * max_tilt_deg
        angle_y = fx_norm * max_tilt_deg
        self.pin_cylinder.rotate(angle_x, 1, 0, 0)
        self.pin_cylinder.rotate(angle_y, 0, 1, 0)
        self.pin_cone.rotate(angle_x, 1, 0, 0)
        self.pin_cone.rotate(angle_y, 0, 1, 0)

        self.base_cylinder.resetTransform()
        self.base_cylinder.translate(0, 0, -0.25)
        self.base_cylinder.rotate(np.degrees(t1), 1, 0, 0)
        self.base_cylinder.rotate(np.degrees(t2), 0, 1, 0)
        self.base_cylinder.rotate(np.degrees(t3), 0, 0, 1)

    def toggle_recording(self):
        if not self.recording:
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
        if not self.recorded_data:
            QMessageBox.warning(self, "No data", "No data to save.")
            return
        options = QFileDialog.Options()
        filename, _ = QFileDialog.getSaveFileName(self, "Save Data", "ft300_data.csv", "CSV Files (*.csv)", options=options)
        if filename:
            with open(filename, 'w', newline='') as f:
                writer = csv.writer(f)
                writer.writerow(['timestamp', 'fx', 'fy', 'fz', 't1', 't2', 't3', 'frequency'])
                writer.writerows(self.recorded_data)
            QMessageBox.information(self, "Saved", f"Data saved to {filename}")

    def toggle_video_recording(self):
        if not self.video_recording:
            self.frames = []
            self.video_recording = True
            self.video_btn.setText("Stop Video")
            self.video_start_time = time.time()
        else:
            self.video_recording = False
            self.video_btn.setText("Record Video")
            if not self.frames:
                QMessageBox.warning(self, "No frames", "No video frames captured.")
                return
            try:
                import imageio
                options = QFileDialog.Options()
                filename, _ = QFileDialog.getSaveFileName(self, "Save Video", "ft300_scene.mp4", "MP4 Files (*.mp4)", options=options)
                if filename:
                    frames_uint8 = [np.clip((frame * 255), 0, 255).astype(np.uint8) for frame in self.frames]
                    imageio.mimsave(filename, frames_uint8, fps=30)
                    QMessageBox.information(self, "Saved", f"Video saved to {filename}")
            except ImportError:
                QMessageBox.warning(self, "Missing package", "Install 'imageio' to enable video export:\npip install imageio")

if __name__ == '__main__':
    sensor_thread = SensorThread()
    sensor_thread.daemon = True
    sensor_thread.start()

    app = QApplication(sys.argv)
    window = Sensor3DVisualizer()
    window.show()
    sys.exit(app.exec_())
