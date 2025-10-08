from selfdrive.ui.widgets.startup_logo import StartupLogoWidget
# --- Tesla-Škoda startup logo ---
startup_logo = StartupLogoWidget()
startup_logo.show()
app.processEvents()   # nech sa logo hneď zobrazí
time.sleep(2.5)       # dĺžka zobrazenia loga (v sekundách)
startup_logo.close()
# --- end of startup logo ---
import sys
from PyQt5.QtWidgets import QApplication, QWidget
from PyQt5.QtGui import QPixmap, QPainter
from PyQt5.QtCore import QTimer

# Cesta k tvojmu logu
LOGO_PATH = "selfdrive/ui/qt/assets/images/tesla_skoda.png"

class TeslaSkodaStartup(QWidget):
    def __init__(self):
        super().__init__()
        self.logo = QPixmap(LOGO_PATH)
        self.setWindowTitle("Tesla-Škoda Startup")
        self.showFullScreen()  # celé okno

    def paintEvent(self, event):
        painter = QPainter(self)
        painter.setRenderHint(QPainter.Antialiasing)
        
        width = self.width()
        height = self.height()

        logo_width = int(width * 0.6)
        logo_height = int(self.logo.height() * (logo_width / self.logo.width()))

        x = (width - logo_width) // 2
        y = (height - logo_height) // 2

        painter.drawPixmap(x, y, logo_width, logo_height, self.logo)

# Spustenie aplikácie
if __name__ == "__main__":
    app = QApplication(sys.argv)
    window = TeslaSkodaStartup()
    window.show()

    # Skryje logo po 3 sekundách
    QTimer.singleShot(3000, window.close)

    sys.exit(app.exec_())
