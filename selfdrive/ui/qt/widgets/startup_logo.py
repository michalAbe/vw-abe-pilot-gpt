from PyQt5.QtWidgets import QWidget, QLabel
from PyQt5.QtGui import QPixmap, QPainter
from PyQt5.QtCore import Qt, QTimer, QPropertyAnimation, pyqtProperty

class FadeLabel(QLabel):
    def __init__(self, parent=None):
        super().__init__(parent)
        self._opacity = 1.0

    def setOpacity(self, opacity):
        self._opacity = opacity
        self.update()

    def getOpacity(self):
        return self._opacity

    opacity = pyqtProperty(float, fget=getOpacity, fset=setOpacity)

    def paintEvent(self, event):
        painter = QPainter(self)
        painter.setOpacity(self._opacity)
        super().paintEvent(event)

class StartupLogoWidget(QWidget):
    def __init__(self, parent=None):
        super().__init__(parent)
        self.setFixedSize(800, 200)
        self.setAttribute(Qt.WA_TranslucentBackground)
        self.setAttribute(Qt.WA_NoSystemBackground, True)

        self.logo_label = FadeLabel(self)
        self.logo_label.setAlignment(Qt.AlignCenter)
        self.logo_label.setGeometry(0, 0, 800, 200)

        pixmap = QPixmap("selfdrive/ui/images/tesla_skoda_logo.png")
        self.logo_label.setPixmap(pixmap.scaled(self.logo_label.size(), Qt.KeepAspectRatio, Qt.SmoothTransformation))

        self.animation = QPropertyAnimation(self.logo_label, b"opacity")
        self.animation.setDuration(1000)
        self.animation.setStartValue(0.0)
        self.animation.setEndValue(1.0)
        self.animation.start()

        QTimer.singleShot(3000, self.fade_out)

    def fade_out(self):
        self.animation.setDirection(QPropertyAnimation.Backward)
        self.animation.start()
        QTimer.singleShot(1000, self.hide)
