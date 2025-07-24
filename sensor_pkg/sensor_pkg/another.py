from PyQt5.QtWidgets import QApplication, QOpenGLWidget
from OpenGL.GL import *
import sys

class GLTest(QOpenGLWidget):
    def initializeGL(self):
        print("OpenGL context initialized")
        print("Vendor:", glGetString(GL_VENDOR))
        print("Renderer:", glGetString(GL_RENDERER))
        print("Version:", glGetString(GL_VERSION))

app = QApplication(sys.argv)
w = GLTest()
w.resize(300, 300)
w.show()
sys.exit(app.exec_())

