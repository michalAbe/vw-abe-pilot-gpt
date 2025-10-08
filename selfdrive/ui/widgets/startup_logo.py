from PyQt5.QtWidgets import QWidget, QLabel
from PyQt5.QtGui import QPixmap
from PyQt5.QtCore import Qt

class StartupLogoWidget(QWidget):
    def __init__(self, parent=None):
        super().__init__(parent)
        
        # Nastavenie veľkosti a pozície widgetu
        self.setFixedSize(300, 100)  # uprav podľa potreby
        self.setAttribute(Qt.WA_TranslucentBackground)
        
        # Label pre obrázok
        self.logo_label = QLabel(self)
        self.logo_label.setAlignment(Qt.AlignCenter)
        
        # Načítanie obrázka
        pixmap = QPixmap("selfdrive/ui/images/tesla_skoda_logo.png")
        self.logo_label.setPixmap(pixmap.scaled(self.size(), Qt.KeepAspectRatio, Qt.SmoothTransformation))
