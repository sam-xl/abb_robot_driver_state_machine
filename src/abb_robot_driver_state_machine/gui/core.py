from __feature__ import snake_case, true_property
from PySide6.QtGui import QGuiApplication
from PySide6.QtQml import QQmlApplicationEngine


class MainWindow:
    def __init__(self) -> None:
        app = QGuiApplication()

        engine = QQmlApplicationEngine()
        engine.quit.connect(app.quit)
        engine.load("./qml/main.qml")

        app.exec()


if __name__ == "__main__":
    window = MainWindow()
