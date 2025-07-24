import sys
import numpy as np
from PyQt5.QtWidgets import QApplication, QMainWindow
from PyQt5.QtCore import QTimer
import pyqtgraph.opengl as gl

class MinimalTester(QMainWindow):
    def __init__(self):
        super().__init__()
        self.setWindowTitle('Minimal OpenGL Test')
        self.view = gl.GLViewWidget()
        self.setCentralWidget(self.view)
        self.view.setCameraPosition(distance=10)

        # Create a simple 3D object
        mesh_data = gl.MeshData.sphere(rows=10, cols=20)
        self.mesh_item = gl.GLMeshItem(
            meshdata=mesh_data,
            smooth=True,
            color=(0.2, 0.6, 1.0, 1.0), # Blue
            shader='shaded'
        )
        # self.view.addItem(self.mesh_item)
        self.mesh_ready = False
        
        # Simple counter for animation
        self.angle = 0

        # Use the showEvent to safely start the timer
        self.timer = QTimer()
        self.timer.timeout.connect(self.update_animation)

    def showEvent(self, event):
        """Called by Qt automatically when the window is shown."""
        print("showEvent triggered. Starting timer.")
        super().showEvent(event)
        if not self.timer.isActive():
            self.timer.start(30) # approx 33 FPS
        if not self.mesh_ready:
            self.view.addItem(self.mesh_item)
            self.mesh_ready = True

    def update_animation(self):
        """Called by the QTimer to update the scene."""
        self.angle += 1
        # Rotate the object
        self.mesh_item.resetTransform()
        self.mesh_item.rotate(self.angle, 0, 1, 0) # Rotate around Y axis
        # This print statement confirms the update loop is running
        if self.angle % 100 == 0:
            print(f"Update loop running, angle: {self.angle}")

if __name__ == '__main__':
    print("Starting application...")
    app = QApplication(sys.argv)
    window = MinimalTester()
    window.show()
    sys.exit(app.exec_())
