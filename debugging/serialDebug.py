import sys
import serial
from PyQt5.QtWidgets import QApplication, QMainWindow, QVBoxLayout, QWidget, QPushButton, QLineEdit, QTextEdit
from PyQt5.QtCore import QThread, pyqtSignal
from matplotlib.figure import Figure
from matplotlib.backends.backend_qt5agg import FigureCanvasQTAgg as FigureCanvas
import numpy as np

DEVICE_PATH = "/dev/cu.usbserial-10"
BAUD_RATE = 9600

class SerialThread(QThread):
    data_received = pyqtSignal(str)

    def __init__(self, port, baud_rate):
        super().__init__()
        self.port = port
        self.baud_rate = baud_rate
        self.running = True

    def run(self):
        with serial.Serial(self.port, self.baud_rate, timeout=1) as ser:
            while self.running:
                if ser.in_waiting:
                    data = ser.readline().decode().strip()
                    self.data_received.emit(data)

    def stop(self):
        self.running = False
        self.wait()

class MainWindow(QMainWindow):
    def __init__(self):
        super().__init__()
        self.setWindowTitle("Serial Data Plotter")
        
        self.main_widget = QWidget(self)
        self.setCentralWidget(self.main_widget)
        layout = QVBoxLayout(self.main_widget)

        self.figure = Figure()
        self.canvas = FigureCanvas(self.figure)
        layout.addWidget(self.canvas)
        self.ax = self.figure.add_subplot(111)
        self.ax.set_title("Real-time Data Plot")
        self.lines, = self.ax.plot([], [], '-')

        self.text_edit = QTextEdit()
        self.text_edit.setReadOnly(True)
        layout.addWidget(self.text_edit)

        self.line_edit = QLineEdit()
        layout.addWidget(self.line_edit)

        self.send_button = QPushButton("Send")
        self.send_button.clicked.connect(self.send_data)
        layout.addWidget(self.send_button)

        self.serial_thread = SerialThread(DEVICE_PATH, BAUD_RATE)
        self.serial_thread.data_received.connect(self.update_plot)
        self.serial_thread.start()

    def update_plot(self, data):
        try:
            # Assuming data is a comma-separated string of numbers
            numbers = list(map(float, data.split(',')))
            self.lines.set_xdata(np.append(self.lines.get_xdata(), len(self.lines.get_xdata())))
            self.lines.set_ydata(np.append(self.lines.get_ydata(), numbers))
            self.ax.relim()
            self.ax.autoscale_view(True,True,True)
            self.canvas.draw()
            self.text_edit.append(data)
        except ValueError:
            pass  # Ignore any conversion errors

    def send_data(self):
        data = self.line_edit.text() + '\n'
        self.serial_thread.serial.write(data.encode())
        self.line_edit.clear()

    def closeEvent(self, event):
        self.serial_thread.stop()
        super().closeEvent(event)

if __name__ == '__main__':
    app = QApplication(sys.argv)
    main = MainWindow()
    main.show()
    sys.exit(app.exec_())


# import sys
# import serial
# from PyQt5.QtWidgets import QApplication, QMainWindow, QVBoxLayout, QTextEdit, QLineEdit, QPushButton, QWidget
# from PyQt5.QtCore import QTimer, QThread, pyqtSignal, Qt
# import matplotlib
# from matplotlib.backends.backend_qt5agg import FigureCanvasQTAgg as FigureCanvas
# from matplotlib.figure import Figure
# matplotlib.use('Qt5Agg')

# LENGTH_LIMIT = 100
# DEVICE_PATH = "/dev/cu.usbserial-10"

# class SerialThread(QThread):
#     data_received = pyqtSignal(str)

#     def __init__(self, parent=None):
#         super().__init__(parent)
#         self.ser = None
#         self.data_dict = {}
#         self.running = False

#     def run(self):
#         self.running = True
#         try:
#             self.ser = serial.Serial(DEVICE_PATH, 9600, timeout=0.1)
#             while self.running:
#                 data = self.ser.readline().decode("utf-8").strip()
#                 if data:
#                     self.parse_data(data)
#                     self.data_received.emit(data)
#         except serial.SerialException as e:
#             print(f"Error opening serial port: {e}")
#         finally:
#             if self.ser:
#                 self.ser.close()

#     def parse_data(self, line):
#         pairs = line.split(', ')
#         for pair in pairs[:-1]:
#             if ':' in pair:
#                 name, value = pair.split(':', 1)
#                 if name not in self.data_dict:
#                     self.data_dict[name] = []
#                 if len(self.data_dict[name]) >= LENGTH_LIMIT:
#                     self.data_dict[name].pop(0)
#                 self.data_dict[name].append(float(value))

#     def write_data(self, message):
#         if self.ser:
#             self.ser.write(message.encode('utf-8'))

#     def stop(self):
#         self.running = False
#         self.wait()

# class MainWindow(QMainWindow):
#     def __init__(self):
#         super().__init__()
#         self.setWindowTitle("Serial Data Plotter")
#         self.resize(800, 600)

#         central_widget = QWidget()
#         layout = QVBoxLayout()

#         self.text_edit = QTextEdit()
#         layout.addWidget(self.text_edit)

#         self.figure = Figure()
#         self.canvas = FigureCanvas(self.figure)
#         layout.addWidget(self.canvas)

#         self.line_edit = QLineEdit()
#         layout.addWidget(self.line_edit)

#         send_button = QPushButton("Send")
#         send_button.clicked.connect(self.send_data)
#         layout.addWidget(send_button)

#         central_widget.setLayout(layout)
#         self.setCentralWidget(central_widget)

#         self.serial_thread = SerialThread()
#         self.serial_thread.data_received.connect(self.update_text)
#         self.serial_thread.start()

#         self.data_dict = self.serial_thread.data_dict
#         self.timer = QTimer()
#         self.timer.timeout.connect(self.update_plot)
#         self.timer.start(50)

#     def update_text(self, data):
#         self.text_edit.append(f"Received: {data}")

#     def update_plot(self):
#         self.figure.clear()
#         ax = self.figure.add_subplot(111)
#         for name, values in self.data_dict.items():
#             ax.plot(range(len(values)), values, label=name)
#         ax.legend()
#         ax.set_title("Live Data Plot")
#         ax.set_xlabel("Index")
#         ax.set_ylabel("Value")
#         self.canvas.draw()

#     def send_data(self):
#         message = self.line_edit.text()
#         self.serial_thread.write_data(message)
#         self.line_edit.clear()

#     def closeEvent(self, event):
#         self.serial_thread.stop()
#         event.accept()

# if __name__ == "__main__":
#     app = QApplication(sys.argv)
#     window = MainWindow()
#     window.show()
#     sys.exit(app.exec_())