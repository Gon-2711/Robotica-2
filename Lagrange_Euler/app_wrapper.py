import os
import sys
from PyQt5.QtWidgets import QApplication, QMainWindow, QVBoxLayout, QWidget
from PyQt5.QtWebEngineWidgets import QWebEngineView
from PyQt5.QtCore import QUrl

def get_html_path():
    if getattr(sys, 'frozen', False):
        base_path = sys._MEIPASS
    else:
        base_path = os.path.dirname(os.path.abspath(__file__))
    return os.path.join(base_path, 'lagrange_euler_app.html')

class MainWindow(QMainWindow):
    def __init__(self):
        super().__init__()
        self.setWindowTitle('Simulador Lagrange-Euler [Web Engine]')
        self.resize(1400, 900)
        
        self.browser = QWebEngineView()
        
        html_file = get_html_path()
        file_url = 'file:///' + html_file.replace('\\', '/')
        self.browser.setUrl(QUrl(file_url))
        
        self.setCentralWidget(self.browser)

if __name__ == '__main__':
    app = QApplication(sys.argv)
    window = MainWindow()
    window.show()
    sys.exit(app.exec_())
