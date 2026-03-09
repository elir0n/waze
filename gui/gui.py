import sys
import os
import subprocess

def main():
    html_path = os.path.join(os.path.dirname(os.path.abspath(__file__)), "map.html")

    # Try PySide6 / PyQt6 WebEngine first (requires QtWebEngine)
    try:
        from PySide6.QtWidgets import QApplication, QMainWindow
        from PySide6.QtWebEngineWidgets import QWebEngineView
        from PySide6.QtCore import QUrl

        class Window(QMainWindow):
            def __init__(self):
                super().__init__()
                self.setWindowTitle("Waze Simulation")
                self.resize(1200, 800)
                self.browser = QWebEngineView()
                self.browser.load(QUrl.fromLocalFile(html_path))
                self.setCentralWidget(self.browser)

        app = QApplication(sys.argv)
        window = Window()
        window.show()
        sys.exit(app.exec())
        return
    except ImportError:
        pass

    # Fallback: open in Windows Chrome via WSL
    chrome = r"/mnt/c/Program Files/Google/Chrome/Application/chrome.exe"
    if os.path.exists(chrome):
        # Convert WSL path to Windows path for Chrome
        wsl_path = subprocess.check_output(["wslpath", "-w", html_path]).decode().strip()
        subprocess.Popen([chrome, wsl_path])
        print(f"Opened map in Chrome: {wsl_path}")
        return

    # Last resort: Python's webbrowser module
    import webbrowser
    webbrowser.open(f"file://{html_path}")
    print(f"Opened map: file://{html_path}")

if __name__ == "__main__":
    main()
