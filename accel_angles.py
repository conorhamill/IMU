# ---------------------------------------------------------------
# accel_angles.py
# Shows roll and pitch computed from the ACCELEROMETER ONLY -
# no gyro, no filter. Just the trigonometry:
#
#   roll  = atan2(ay, az)
#   pitch = atan2(-ax, sqrt(ay^2 + az^2))
#
# The idea: a stationary accel only feels gravity (1 g, straight
# down), reported along the sensor's own axes. How that 1 g splits
# between the axes encodes the tilt, and atan2 works backwards
# from the components to the angle.
#
# What to expect when you run it:
#   - correct angles with NO drift, however long you leave it
#   - but noisy (the trace jitters by a degree or two)
#   - and easily fooled: shake or move the board and the "angles"
#     jump around even though the tilt hasn't changed
# Those weaknesses are what the gyro fixes in a fusion filter.
#
# Uses the same Arduino sketch as raw_data.py.
# Requirements:  pip install pyserial matplotlib
# ---------------------------------------------------------------

import tkinter as tk
import math
import serial
from matplotlib.figure import Figure
from matplotlib.backends.backend_tkagg import FigureCanvasTkAgg

PORT = "COM3"      # <-- change this to your Arduino's port
BAUD = 115200      # must match Serial.begin() in the sketch

ser = serial.Serial(PORT, BAUD, timeout=0.1)

ACCEL_SCALE = 16384.0   # raw counts per g (at the +/-2 g range)

names = ["Roll", "Pitch"]
data = [[], []]         # angle history, grows until Reset

# --- Build the window ------------------------------------------

window = tk.Tk()
window.title("Accelerometer-only roll and pitch")

fig = Figure(figsize=(8, 4))
plot = fig.add_subplot(1, 1, 1)
plot.set_title("Roll and pitch from accel alone")
plot.set_ylabel("degrees")
plot.set_xlabel("Sample number")

lines = []
for name in names:
    line, = plot.plot([], [], label=name)
    lines.append(line)
plot.legend(loc="upper left")
fig.tight_layout()

canvas = FigureCanvasTkAgg(fig, master=window)
canvas.get_tk_widget().pack(fill="both", expand=True)

def reset():
    for d in data:
        d.clear()

tk.Button(window, text="Reset", font=("Arial", 12),
          command=reset).pack(pady=5)

# --- Read serial data and redraw the chart ---------------------

def update():
    while ser.in_waiting:
        line = ser.readline().decode(errors="ignore").strip()
        parts = line.split(",")
        if len(parts) == 6:                  # only use complete lines
            try:
                values = [int(p) for p in parts]
            except ValueError:
                continue                     # skip garbled lines

            # only the first three values (accel) are needed here -
            # the gyro values are ignored completely
            ax = values[0] / ACCEL_SCALE
            ay = values[1] / ACCEL_SCALE
            az = values[2] / ACCEL_SCALE

            # roll: how gravity splits between the y and z axes
            roll = math.degrees(math.atan2(ay, az))

            # pitch: the x axis against everything else - when the
            # board pitches, gravity appears on x while the rest
            # stays spread over y and z combined
            pitch = math.degrees(math.atan2(-ax,
                        math.sqrt(ay * ay + az * az)))

            data[0].append(roll)
            data[1].append(pitch)

    # hand the plot lines their new data and show the latest
    # value in the legend
    x = range(len(data[0]))
    for line, name, d in zip(lines, names, data):
        line.set_data(x, d)
        if d:
            line.set_label(f"{name}: {d[-1]:.1f}")
    plot.legend(loc="upper left")

    plot.relim()
    plot.autoscale_view()
    canvas.draw()

    window.after(100, update)   # run again in 100 ms

update()
window.mainloop()
ser.close()
