# # import serial
# # import threading
# # import matplotlib.pyplot as plt
# # from matplotlib.backends.backend_tkagg import FigureCanvasTkAgg
# # import tkinter as tk
# # from tkinter import scrolledtext

# # LENGTH_LIMIT = 1000
# # DEVICE_PATH = "/dev/cu.usbserial-10"

# # stop_threads = False

# # try:
# #     ser = serial.Serial(DEVICE_PATH, 9600, timeout=0.5)
# # except serial.SerialException as e:
# #     print(f"Error opening serial port: {e}")
# #     exit()

# # def parse_data(line, data_dict):
# #     # Split the line into name-value pairs
# #     pairs = line.split(', ')
# #     for pair in pairs:
# #         if ':' in pair:
# #             name, value = pair.split(':', 1)
# #             if name not in data_dict:
# #                 data_dict[name] = []
# #             if len(data_dict[name]) >= LENGTH_LIMIT:
# #                 data_dict[name].pop(0)
# #             data_dict[name].append(float(value))

# # def update_plot(fig, ax, data_dict, canvas):
# #     ax.clear()
# #     for name, values in data_dict.items():
# #         ax.plot(range(len(values)), values, label=name)
# #     ax.legend()
# #     ax.set_title("Live Data Plot")
# #     ax.set_xlabel("Index")
# #     ax.set_ylabel("Value")
# #     fig.canvas.draw()
# #     canvas.draw_idle()  # Update the canvas

# # def update_gui(message, text_area):
# #     text_area.insert(tk.END, message + "\n")

# # def read_serial(fig, ax, data_dict, text_area, canvas):
# #     while not stop_threads:
# #         try:
# #             data = ser.readline().decode("utf-8")
# #             if data:
# #                 root.after(0, update_gui, "Received: " + data, text_area)
# #                 parse_data(data, data_dict)
# #                 root.after(0, update_plot, fig, ax, data_dict, canvas)
# #                 root.update()  # Ensure GUI updates are processed
# #                 plt.pause(0.01)  # Pause to allow the plot to update
# #         except Exception as e:
# #             print(f"Error reading from serial port: {e}")

# # def write_serial(entry_field):
# #     message = entry_field.get()
# #     ser.write(message.encode('utf-8'))
# #     entry_field.delete(0, tk.END)

# # def on_close():
# #     global stop_threads
# #     stop_threads = True
# #     read_thread.join(timeout=2)
# #     ser.close()
# #     root.destroy()

# # # Create the GUI
# # root = tk.Tk()

# # text_area = scrolledtext.ScrolledText(root)
# # text_area.pack()

# # entry_field = tk.Entry(root)
# # entry_field.pack()

# # send_button = tk.Button(root, text="Send", command=lambda: write_serial(entry_field))
# # send_button.pack()

# # # Matplotlib integration
# # fig, ax = plt.subplots()
# # canvas = FigureCanvasTkAgg(fig, master=root)
# # canvas_widget = canvas.get_tk_widget()
# # canvas_widget.pack()

# # # Data dictionary to store received values
# # data_dict = {}

# # # Create and start the read thread
# # read_thread = threading.Thread(target=lambda: read_serial(fig, ax, data_dict, text_area, canvas))
# # read_thread.start()

# # root.protocol("WM_DELETE_WINDOW", on_close)
# # root.mainloop()


# import serial
# import threading
# import matplotlib.pyplot as plt
# from matplotlib.backends.backend_tkagg import FigureCanvasTkAgg
# from matplotlib.animation import FuncAnimation
# import tkinter as tk
# from tkinter import scrolledtext

# LENGTH_LIMIT = 1000
# DEVICE_PATH = "/dev/cu.usbserial-10"

# stop_threads = False

# try:
#     ser = serial.Serial(DEVICE_PATH, 9600, timeout=0.5)
# except serial.SerialException as e:
#     print(f"Error opening serial port: {e}")
#     exit()

# def parse_data(line, data_dict):
#     # Split the line into name-value pairs
#     pairs = line.split(', ')
#     for pair in pairs:
#         if ':' in pair:
#             name, value = pair.split(':', 1)
#             if name not in data_dict:
#                 data_dict[name] = []
#             if len(data_dict[name]) >= LENGTH_LIMIT:
#                 data_dict[name].pop(0)
#             data_dict[name].append(float(value))

# def update_plot(frame, fig, ax, data_dict, canvas):
#     ax.clear()
#     for name, values in data_dict.items():
#         ax.plot(range(len(values)), values, label=name)
#     ax.legend()
#     ax.set_title("Live Data Plot")
#     ax.set_xlabel("Index")
#     ax.set_ylabel("Value")
#     fig.canvas.draw()
#     canvas.draw_idle()  # Update the canvas

# def update_gui(message, text_area):
#     text_area.insert(tk.END, message + "\n")

# def read_serial(frame, fig, ax, data_dict, text_area, canvas):
#     while not stop_threads:
#         try:
#             data = ser.readline().decode("utf-8")
#             if data:
#                 root.after(0, update_gui, "Received: " + data, text_area)
#                 parse_data(data, data_dict)
#         except Exception as e:
#             print(f"Error reading from serial port: {e}")

# # Create the GUI
# root = tk.Tk()

# text_area = scrolledtext.ScrolledText(root)
# text_area.pack()

# # Matplotlib integration
# fig, ax = plt.subplots()
# canvas = FigureCanvasTkAgg(fig, master=root)
# canvas_widget = canvas.get_tk_widget()
# canvas_widget.pack()

# # Data dictionary to store received values
# data_dict = {}

# # Create and start the animation
# ani = FuncAnimation(fig, update_plot, fargs=(fig, ax, data_dict, canvas), interval=10)
# ani.event_source.stop()

# def start_animation():
#     ani.event_source.start()

# def stop_animation():
#     ani.event_source.stop()

# # Create and start the read thread
# read_thread = threading.Thread(target=lambda: read_serial(0, fig, ax, data_dict, text_area, canvas))
# read_thread.start()

# start_button = tk.Button(root, text="Start Animation", command=start_animation)
# start_button.pack()

# stop_button = tk.Button(root, text="Stop Animation", command=stop_animation)
# stop_button.pack()

# root.mainloop()

import serial
import threading
import matplotlib.pyplot as plt
from matplotlib.backends.backend_tkagg import FigureCanvasTkAgg
from matplotlib.animation import FuncAnimation
import tkinter as tk
from tkinter import scrolledtext

LENGTH_LIMIT = 1000
DEVICE_PATH = "/dev/cu.usbserial-10"

stop_threads = False

try:
    ser = serial.Serial(DEVICE_PATH, 9600, timeout=0.5)
except serial.SerialException as e:
    print(f"Error opening serial port: {e}")
    exit()

def parse_data(line, data_dict):
    # Split the line into name-value pairs
    pairs = line.split(', ')
    for pair in pairs:
        if ':' in pair:
            name, value = pair.split(':', 1)
            if name not in data_dict:
                data_dict[name] = []
            if len(data_dict[name]) >= LENGTH_LIMIT:
                data_dict[name].pop(0)
            data_dict[name].append(float(value))

def update_plot(frame, fig, ax, data_dict, canvas):
    ax.clear()
    for name, values in data_dict.items():
        ax.plot(range(len(values)), values, label=name)
    ax.legend()
    ax.set_title("Live Data Plot")
    ax.set_xlabel("Index")
    ax.set_ylabel("Value")
    fig.canvas.draw()
    canvas.draw_idle()  # Update the canvas

def update_gui(message, text_area):
    text_area.insert(tk.END, message + "\n")

def read_serial(frame, fig, ax, data_dict, text_area, canvas):
    while not stop_threads:
        try:
            data = ser.readline().decode("utf-8")
            if data:
                root.after(0, update_gui, "Received: " + data, text_area)
                parse_data(data, data_dict)
        except Exception as e:
            print(f"Error reading from serial port: {e}")
    ser.close()
    plt.close()

def write_serial(entry_field):
    message = entry_field.get()
    ser.write(message.encode('utf-8'))
    entry_field.delete(0, tk.END)

# Create the GUI
root = tk.Tk()

text_area = scrolledtext.ScrolledText(root)
text_area.pack()

# Matplotlib integration
fig, ax = plt.subplots()
canvas = FigureCanvasTkAgg(fig, master=root)
canvas_widget = canvas.get_tk_widget()
canvas_widget.pack()

# Data dictionary to store received values
data_dict = {}

# Create and start the animation
ani = FuncAnimation(fig, update_plot, fargs=(fig, ax, data_dict, canvas), interval=10)
ani.event_source.stop()

def start_animation():
    ani.event_source.start()

def stop_animation():
    ani.event_source.stop()

# Create and start the read thread
read_thread = threading.Thread(target=lambda: read_serial(0, fig, ax, data_dict, text_area, canvas))
read_thread.start()

start_button = tk.Button(root, text="Start Animation", command=start_animation)
start_button.pack()

stop_button = tk.Button(root, text="Stop Animation", command=stop_animation)
stop_button.pack()

entry_field = tk.Entry(root)
entry_field.pack()

send_button = tk.Button(root, text="Send", command=lambda: write_serial(entry_field))
send_button.pack()

root.mainloop()

