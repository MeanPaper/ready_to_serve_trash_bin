import sys
import serial
from PyQt5.QtCore import QTimer, Qt
from PyQt5.QtWidgets import QApplication, QMainWindow, QVBoxLayout, QWidget, QPushButton, QLineEdit, QTextEdit
from matplotlib.figure import Figure
from matplotlib.backends.backend_qt5agg import FigureCanvasQTAgg as FigureCanvas
import numpy as np

DATA_LEN_LIM = 200

class SerialReader:
    def __init__(self, port, baud_rate):
        self.port = port
        self.baud_rate = baud_rate
        self.ser = None

    def start(self):
        self.ser = serial.Serial(self.port, self.baud_rate, timeout=1)

    def read_data(self):
        if self.ser and self.ser.in_waiting:
            return self.ser.readline().decode().strip()
        return None

    def stop(self):
        if self.ser and self.ser.is_open:
            self.ser.close()

class MainWindow(QMainWindow):
    def __init__(self):
        super(MainWindow, self).__init__()
        self.setWindowTitle("Serial Data Plotter")

        self.main_widget = QWidget(self)
        self.setCentralWidget(self.main_widget)
        layout = QVBoxLayout(self.main_widget)

        self.figure = Figure()
        self.canvas = FigureCanvas(self.figure)
        layout.addWidget(self.canvas)
        self.ax = self.figure.add_subplot(111)
        self.ax.set_title("Real-time Data Plot")
        self.data_dict = {}
        self.lines = {}

        self.text_edit = QTextEdit()
        self.text_edit.setReadOnly(True)
        layout.addWidget(self.text_edit)

        self.line_edit = QLineEdit()
        layout.addWidget(self.line_edit)

        self.send_button = QPushButton("Send")
        self.send_button.clicked.connect(self.send_data)
        layout.addWidget(self.send_button)

        self.serial_reader = SerialReader("/dev/cu.usbserial-10", 9600)

        self.timer = QTimer(self)
        self.timer.timeout.connect(self.check_serial)
        self.timer.start(10)  # Adjust the timeout as needed

    def check_serial(self):
        data = self.serial_reader.read_data()
        if data:
            print(f"Received from serial: {data}")
            self.parse_and_update_data(data)

            for key, values in self.data_dict.items():
                if key not in self.lines:
                    self.lines[key], = self.ax.plot([], [], label=key)
                x_data = list(range(1, len(values) + 1))
                self.lines[key].set_data(x_data, values)

            self.ax.relim()
            self.ax.autoscale_view(True, True, True)

            if self.ax.get_legend() is None:
                self.ax.legend()

            self.canvas.draw()
            self.text_edit.append(data)

    def parse_and_update_data(self, data):
        parts = data.split(',')
        for part in parts:
            if ':' in part:
                try:
                    key, value = part.split(':')
                    key = key.strip()
                    value = float(value.strip())
                    if key not in self.data_dict:
                        self.data_dict[key] = [value]
                    else:
                        if len(self.data_dict[key]) > DATA_LEN_LIM:
                            self.data_dict[key].pop(0)
                        self.data_dict[key].append(value)
                except ValueError as e:
                    print(f"Error parsing data '{part}': {e}")
                    continue

    def send_data(self):
        try:
            data = self.line_edit.text() + '\n'
            if self.serial_reader.ser and self.serial_reader.ser.is_open:
                self.serial_reader.ser.write(data.encode())
            self.line_edit.clear()
        except Exception as e:
            print(f"Error during serial write: {e}")

    def closeEvent(self, event):
        self.serial_reader.stop()
        super().closeEvent(event)

if __name__ == '__main__':
    app = QApplication(sys.argv)
    main = MainWindow()
    main.show()
    sys.exit(app.exec_())
