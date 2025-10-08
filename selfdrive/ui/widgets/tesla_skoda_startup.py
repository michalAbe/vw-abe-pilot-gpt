from PyQt5.QtWidgets import QWidget
from PyQt5.QtGui import QPixmap, QPainter

class TeslaSkodaStartup(QWidget):
    def __init__(self, parent=None):
        super().__init__(parent)
        self.logo = QPixmap("assets/images/tesla_skoda.png")  # cesta k tvojmu logu

    def paintEvent(self, event):
        painter = QPainter(self)
        painter.setRenderHint(QPainter.Antialiasing)
        
        width = self.width()
        height = self.height()

        # veľkosť loga (60% šírky obrazovky)
        logo_width = int(width * 0.6)
        logo_height = int(self.logo.height() * (logo_width / self.logo.width()))

        # stred obrazovky
        x = (width - logo_width) // 2
        y = (height - logo_height) // 2

        painter.drawPixmap(x, y, logo_width, logo_height, self.logo)
