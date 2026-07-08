# ---------------------------------------------------------------
# raw_data.py
# GUI that plots accel/gyro values from the Arduino on a chart,
# converted to real units (g and deg/s).
#
# The Arduino sends one line per sample over the serial port:
#     ax,ay,az,gx,gy,gz
#
# The chart keeps ALL data since the start (no moving window).
# The Reset button clears the chart and starts again from zero.
#
# Requirements:  pip install pyserial matplotlib
#
# Change PORT below to match your Arduino
# (look in the Arduino IDE under Tools > Port, e.g. "COM3")
# ---------------------------------------------------------------

import tkinter as tk
import math
import serial
from matplotlib.figure import Figure
from matplotlib.backends.backend_tkagg import FigureCanvasTkAgg

PORT = "COM3"      # <-- change this to your Arduino's port
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

# --- Madgwick filter ---------------------------------------------
# Tracks orientation as a quaternion: four numbers [q0, q1, q2, q3]
# that encode a single rotation (an axis and an angle) from the
# world frame to the sensor frame. Unlike roll/pitch/yaw there is
# no rotation sequence, so no gimbal lock and no small-angle limit.
#
# Each update does two things:
#   1) rotates the quaternion by what the gyro measured (fast, drifts)
#   2) nudges it so its predicted gravity direction matches the
#      accelerometer (slow, drift-free) - same fusion idea as the
#      complementary filter, in quaternion form.

BETA = 0.1      # correction strength (the "2%" knob of this filter)
DT = 0.05       # time between samples in seconds (delay(50) in the sketch)

q = [1.0, 0.0, 0.0, 0.0]   # start as "no rotation at all"

angle_names = ["Roll", "Pitch", "Yaw"]
angle_data = [[], [], []]     # history of the estimates, for plotting

# --- Data storage ----------------------------------------------
# One list per signal. They grow forever until Reset is pressed.

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
window.title("MPU-9250 Raw Data")

# A matplotlib figure with two plots stacked vertically:
# accel on top, gyro underneath.
fig = Figure(figsize=(8, 8))
accel_plot = fig.add_subplot(3, 1, 1)   # (rows, columns, position)
gyro_plot = fig.add_subplot(3, 1, 2)
angle_plot = fig.add_subplot(3, 1, 3)

accel_plot.set_title("Accelerometer")
accel_plot.set_ylabel("g")
gyro_plot.set_title("Gyroscope")
gyro_plot.set_ylabel("deg/s")
angle_plot.set_title("Orientation (Madgwick filter)")
angle_plot.set_ylabel("degrees")
angle_plot.set_xlabel("Sample number")

# Create one empty line per signal. We keep the line objects in a
# list so we can give them new data later without rebuilding the plot.
lines = []
for i in range(3):
    line, = accel_plot.plot([], [], label=names[i])
    lines.append(line)
for i in range(3, 6):
    line, = gyro_plot.plot([], [], label=names[i])
    lines.append(line)
angle_lines = []
for name in angle_names:
    line, = angle_plot.plot([], [], label=name)
    angle_lines.append(line)

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
    for d in angle_data:
        d.clear()
    q[:] = [1.0, 0.0, 0.0, 0.0]      # back to "no rotation"
    gyro_bias[:] = [0.0, 0.0, 0.0]   # forget the calibration too
    cal_countdown = 0                # cancel calibration if one is running
    status.config(text="Not calibrated")

def calibrate():
    global cal_countdown
    cal_sums[:] = [0.0, 0.0, 0.0]    # start the totals from zero
    gyro_bias[:] = [0.0, 0.0, 0.0]   # remove any previous calibration
    cal_countdown = CAL_SAMPLES      # update() sees this and starts collecting
    status.config(text="Calibrating - keep the sensor still...")

# a frame holds the two buttons side by side
button_row = tk.Frame(window)
button_row.pack(pady=5)

tk.Button(button_row, text="Reset", font=("Arial", 12),
          command=reset).pack(side="left", padx=5)
tk.Button(button_row, text="Calibrate gyro", font=("Arial", 12),
          command=calibrate).pack(side="left", padx=5)

status = tk.Label(window, text="Not calibrated", font=("Arial", 11))
status.pack(pady=(0, 5))

# --- The Madgwick update, once per sample ------------------------

def madgwick_update(gx, gy, gz, ax, ay, az):
    q0, q1, q2, q3 = q          # unpack the current orientation

    # ---- 1) gyro part: how fast is the quaternion changing? ----
    # This is the quaternion version of "angle += rate * dt".
    # It converts the three body-axis rates into the rate of
    # change of all four quaternion numbers (0.5 * q x omega).
    qdot0 = 0.5 * (-q1 * gx - q2 * gy - q3 * gz)
    qdot1 = 0.5 * ( q0 * gx + q2 * gz - q3 * gy)
    qdot2 = 0.5 * ( q0 * gy - q1 * gz + q3 * gx)
    qdot3 = 0.5 * ( q0 * gz + q1 * gy - q2 * gx)

    # ---- 2) accel part: which way does the estimate need nudging?
    # Only the DIRECTION of the accel matters, so normalise it
    # to length 1 (skip the correction if the reading is zero).
    norm = math.sqrt(ax * ax + ay * ay + az * az)
    if norm > 0:
        ax, ay, az = ax / norm, ay / norm, az / norm

        # f = (gravity direction the quaternion PREDICTS we should
        #      be measuring) minus (what the accel ACTUALLY measured).
        # If the estimate were perfect, f would be zero.
        f1 = 2 * (q1 * q3 - q0 * q2) - ax
        f2 = 2 * (q0 * q1 + q2 * q3) - ay
        f3 = 2 * (0.5 - q1 * q1 - q2 * q2) - az

        # s = the gradient: the direction in quaternion space that
        # reduces that error fastest (Jacobian transposed times f)
        s0 = -2 * q2 * f1 + 2 * q1 * f2
        s1 =  2 * q3 * f1 + 2 * q0 * f2 - 4 * q1 * f3
        s2 = -2 * q0 * f1 + 2 * q3 * f2 - 4 * q2 * f3
        s3 =  2 * q1 * f1 + 2 * q2 * f2

        # normalise the gradient so BETA alone sets the strength
        norm = math.sqrt(s0 * s0 + s1 * s1 + s2 * s2 + s3 * s3)
        if norm > 0:
            # blend: follow the gyro, minus a small step downhill
            # on the accel error (this is the fusion, one line)
            qdot0 -= BETA * s0 / norm
            qdot1 -= BETA * s1 / norm
            qdot2 -= BETA * s2 / norm
            qdot3 -= BETA * s3 / norm

    # ---- 3) integrate and renormalise ---------------------------
    q0 += qdot0 * DT
    q1 += qdot1 * DT
    q2 += qdot2 * DT
    q3 += qdot3 * DT

    # keep the quaternion length exactly 1 so it stays a pure
    # rotation (integration errors slowly stretch it otherwise)
    norm = math.sqrt(q0 * q0 + q1 * q1 + q2 * q2 + q3 * q3)
    q[0], q[1], q[2], q[3] = q0 / norm, q1 / norm, q2 / norm, q3 / norm

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

            # ---- Madgwick filter -------------------------------
            # feed it bias-corrected gyro in rad/s (the maths
            # needs radians) and the accel in g
            madgwick_update(math.radians(gyro[0] - gyro_bias[0]),
                            math.radians(gyro[1] - gyro_bias[1]),
                            math.radians(gyro[2] - gyro_bias[2]),
                            accel[0], accel[1], accel[2])

            # extract roll/pitch/yaw from the quaternion, just
            # for display (standard conversion formulas)
            q0, q1, q2, q3 = q
            roll = math.degrees(math.atan2(2 * (q0 * q1 + q2 * q3),
                                           1 - 2 * (q1 * q1 + q2 * q2)))
            pitch = math.degrees(math.asin(
                        max(-1.0, min(1.0, 2 * (q0 * q2 - q3 * q1)))))
            yaw = math.degrees(math.atan2(2 * (q0 * q3 + q1 * q2),
                                          1 - 2 * (q2 * q2 + q3 * q3)))

            angle_data[0].append(roll)
            angle_data[1].append(pitch)
            angle_data[2].append(yaw)

    # Give each plot line its new data.
    # x values are just the sample numbers 0, 1, 2, ...
    x = range(len(data[0]))
    for line, d in zip(lines, data):
        line.set_data(x, d)#]
    xa = range(len(angle_data[0]))
    for line, d in zip(angle_lines, angle_data):
        line.set_data(xa, d)

    # Put the latest value in each legend entry, e.g. "Accel Z: 0.98".
    # d[-1] is the last (newest) item in the list.
    for line, name, d in zip(lines, names, data):
        if d:                                # skip if list is empty
            line.set_label(f"{name}: {d[-1]:.2f}")
    accel_plot.legend(loc="upper left")      # rebuild the legends so
    gyro_plot.legend(loc="upper left")       # they show the new labels
    for line, name, d in zip(angle_lines, angle_names, angle_data):
        if d:
            line.set_label(f"{name}: {d[-1]:.1f}")
    angle_plot.legend(loc="upper left")

    # Rescale the axes so all the data fits, then redraw
    accel_plot.relim()
    accel_plot.autoscale_view()
    gyro_plot.relim()
    gyro_plot.autoscale_view()
    angle_plot.relim()
    angle_plot.autoscale_view()
    canvas.draw()

    # run this function again in 100 ms (this is how tkinter
    # does repeated tasks without freezing the window)
    window.after(100, update)

update()            # start the update loop
window.mainloop()   # show the window (runs until you close it)

ser.close()         # tidy up the serial port when the window closes
