# ---------------------------------------------------------------
# raw_data_capture.py
# GUI that plots accel/gyro values from the Arduino on a chart,
# converted to real units (g and deg/s). No filtering - raw data
# only, with a Save button to export everything to a CSV file.
#
# The Arduino sends one line per sample over the serial port:
#     ax,ay,az,gx,gy,gz
#
# The chart keeps ALL data since the start (no moving window).
# The Reset button clears the chart and starts again from zero.
# The Save button writes the current data to a CSV file.
#
# Requirements:  pip install pyserial matplotlib
#
# Change PORT below to match your Arduino
# (look in the Arduino IDE under Tools > Port, e.g. "COM3")
# ---------------------------------------------------------------

import csv
import tkinter as tk
from tkinter import filedialog, messagebox
import serial
from matplotlib.figure import Figure
from matplotlib.backends.backend_tkagg import FigureCanvasTkAgg

PORT = "COM10"     # <-- change this to your Arduino's port
BAUD = 115200      # must match Serial.begin() in the sketch

# timeout=0.1 means serial reads give up after 0.1 s instead of
# freezing the GUI if no data arrives
ser = serial.Serial(PORT, BAUD, timeout=0.1)

# --- Conversion factors (from the MPU-9250 datasheet) -----------
# The sensor sends raw 16-bit counts. At the default ranges:
#   accel range +/-2 g      -> 16384 counts per g
#   gyro  range +/-250 deg/s -> 131 counts per deg/s
# Dividing by these turns counts into real units.

ACCEL_SCALE = 16384.0   # counts per g
GYRO_SCALE = 131.0      # counts per deg/s

# --- Data storage ----------------------------------------------
# One list per signal. They grow forever until Reset is pressed.
# Gyro values are stored with the calibration bias subtracted
# (the bias is 0,0,0 until a calibration has been done).

names = ["Accel X", "Accel Y", "Accel Z", "Gyro X", "Gyro Y", "Gyro Z"]
data = [[], [], [], [], [], []]   # data[0] is Accel X, etc.

# --- Gyro calibration ------------------------------------------
# A stationary gyro should read 0 deg/s, so whatever it actually
# reads is bias. Calibrating = averaging readings while the sensor
# is still, then subtracting that average from every later reading.

CAL_SAMPLES = 100          # how many samples to average (~5 s at 20 Hz)

gyro_bias = [0.0, 0.0, 0.0]   # subtracted from every gyro reading
cal_sums = [0.0, 0.0, 0.0]    # running totals while calibrating
cal_countdown = 0             # samples still to collect (0 = not calibrating)

# --- Build the window ------------------------------------------

window = tk.Tk()
window.title("MPU-9250 Raw Data Capture")

# A matplotlib figure with two plots stacked vertically:
# accel on top, gyro underneath.
fig = Figure(figsize=(8, 6))
accel_plot = fig.add_subplot(2, 1, 1)   # (rows, columns, position)
gyro_plot = fig.add_subplot(2, 1, 2)

accel_plot.set_title("Accelerometer")
accel_plot.set_ylabel("g")
gyro_plot.set_title("Gyroscope")
gyro_plot.set_ylabel("deg/s")
gyro_plot.set_xlabel("Sample number")

# Create one empty line per signal. We keep the line objects in a
# list so we can give them new data later without rebuilding the plot.
lines = []
for i in range(3):
    line, = accel_plot.plot([], [], label=names[i])
    lines.append(line)
for i in range(3, 6):
    line, = gyro_plot.plot([], [], label=names[i])
    lines.append(line)

accel_plot.legend(loc="upper left")
gyro_plot.legend(loc="upper left")
fig.tight_layout()

# This puts the matplotlib figure inside the tkinter window
canvas = FigureCanvasTkAgg(fig, master=window)
canvas.get_tk_widget().pack(fill="both", expand=True)

# --- Buttons and status text ------------------------------------

def reset():
    global cal_countdown
    for d in data:
        d.clear()                    # empty every list - chart starts over
    gyro_bias[:] = [0.0, 0.0, 0.0]   # forget the calibration too
    cal_countdown = 0                # cancel calibration if one is running
    status.config(text="Not calibrated")

def calibrate():
    global cal_countdown
    cal_sums[:] = [0.0, 0.0, 0.0]    # start the totals from zero
    gyro_bias[:] = [0.0, 0.0, 0.0]   # remove any previous calibration
    cal_countdown = CAL_SAMPLES      # update() sees this and starts collecting
    status.config(text="Calibrating - keep the sensor still...")

def save():
    # Nothing to save if no data has arrived yet
    if not data[0]:
        messagebox.showinfo("Save", "No data to save yet.")
        return

    # Ask the user where to put the file
    path = filedialog.asksaveasfilename(
        defaultextension=".csv",
        filetypes=[("CSV files", "*.csv"), ("All files", "*.*")],
        title="Save data as CSV")
    if not path:                     # user pressed Cancel
        return

    # One row per sample: sample number, then the six signals.
    # Accel is in g, gyro in deg/s (bias-corrected, as plotted).
    with open(path, "w", newline="") as f:
        writer = csv.writer(f)
        writer.writerow(["sample"] + names)
        for i in range(len(data[0])):
            writer.writerow([i] + [d[i] for d in data])

    status.config(text=f"Saved {len(data[0])} samples to {path}")

# a frame holds the buttons side by side
button_row = tk.Frame(window)
button_row.pack(pady=5)

tk.Button(button_row, text="Reset", font=("Arial", 12),
          command=reset).pack(side="left", padx=5)
tk.Button(button_row, text="Calibrate gyro", font=("Arial", 12),
          command=calibrate).pack(side="left", padx=5)
tk.Button(button_row, text="Save CSV", font=("Arial", 12),
          command=save).pack(side="left", padx=5)

status = tk.Label(window, text="Not calibrated", font=("Arial", 11))
status.pack(pady=(0, 5))

# --- Read serial data and redraw the chart ---------------------

def update():
    global cal_countdown

    # Read every complete line waiting in the serial buffer.
    # (The Arduino may have sent several lines since we last looked -
    # reading them all keeps the chart from lagging behind.)
    while ser.in_waiting:
        line = ser.readline().decode(errors="ignore").strip()
        parts = line.split(",")
        if len(parts) == 6:                  # only use complete lines
            try:
                values = [int(p) for p in parts]
            except ValueError:
                continue                     # skip garbled lines

            # convert raw counts to real units:
            # first three values are accel (-> g),
            # last three are gyro (-> deg/s)
            accel = [values[i] / ACCEL_SCALE for i in range(3)]
            gyro = [values[i] / GYRO_SCALE for i in range(3, 6)]

            # If we are calibrating, add this gyro sample to the totals.
            # When enough samples are collected, the average is the bias.
            if cal_countdown > 0:
                for i in range(3):
                    cal_sums[i] += gyro[i]
                cal_countdown -= 1
                if cal_countdown == 0:       # finished - work out the average
                    for i in range(3):
                        gyro_bias[i] = cal_sums[i] / CAL_SAMPLES
                    status.config(text="Calibrated - gyro bias: "
                                        f"{gyro_bias[0]:.2f}, "
                                        f"{gyro_bias[1]:.2f}, "
                                        f"{gyro_bias[2]:.2f} deg/s")

            # store the values (gyro with its bias subtracted -
            # the bias is 0,0,0 until a calibration has been done)
            for i in range(3):
                data[i].append(accel[i])
                data[i + 3].append(gyro[i] - gyro_bias[i])

    # Give each plot line its new data.
    # x values are just the sample numbers 0, 1, 2, ...
    x = range(len(data[0]))
    for line, d in zip(lines, data):
        line.set_data(x, d)

    # Put the latest value in each legend entry, e.g. "Accel Z: 0.98".
    # d[-1] is the last (newest) item in the list.
    for line, name, d in zip(lines, names, data):
        if d:                                # skip if list is empty
            line.set_label(f"{name}: {d[-1]:.2f}")
    accel_plot.legend(loc="upper left")      # rebuild the legends so
    gyro_plot.legend(loc="upper left")       # they show the new labels

    # Rescale the axes so all the data fits, then redraw
    accel_plot.relim()
    accel_plot.autoscale_view()
    gyro_plot.relim()
    gyro_plot.autoscale_view()
    canvas.draw()

    # run this function again in 100 ms (this is how tkinter
    # does repeated tasks without freezing the window)
    window.after(100, update)

update()            # start the update loop
window.mainloop()   # show the window (runs until you close it)

ser.close()         # tidy up the serial port when the window closes
