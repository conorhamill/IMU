# ---------------------------------------------------------------
# theta_3_limb.py
# GUI that computes the joint angle theta of THREE robot limbs,
# each with its own MPU-9250, read through a TCA9548A mux by the
# raw_data_3_limb.ino sketch. Same pipeline as theta_single_limb.py
# run independently per limb:
#
#   1) gyro bias calibration        (Calibration section)
#   2) Madgwick 6-DOF filter -> q   (Madgwick Filter section)
#   3) theta from gravity in the    (Joint Angle Calculation)
#      e1-e2 plane via atan2
#
# Only the three theta values are plotted (no accel/gyro plots).
#
# The Arduino sends one line per sample with 18 values:
#     ax1,ay1,az1,gx1,gy1,gz1,ax2,...,gz2,ax3,...,gz3
#
# Requirements:  pip install pyserial matplotlib
# Change PORT below to match your Arduino.
# ---------------------------------------------------------------

import tkinter as tk
import math
import serial
from matplotlib.figure import Figure
from matplotlib.backends.backend_tkagg import FigureCanvasTkAgg

PORT = "COM10"     # <-- change this to your Arduino's port
BAUD = 115200      # must match Serial.begin() in the sketch

NUM_LIMBS = 3

# --- Rotation axes (provided manually) ---------------------------
# Unit axis of rotation of each limb in its SENSOR BODY frame,
# from the axis determination procedure. Re-normalised below.
# Note: flipping the sign of an axis reverses the direction in
# which that limb's theta increases.

NHATS = [
    [-0.700, -0.714, -0.013],   # limb 1
    [-0.717, -0.697, -0.001],   # limb 2
    [-0.707, -0.706, -0.027],   # limb 3
]

# Constant added to each limb's theta (degrees) to set where zero is.
# Use this when a limb reads a fixed amount out (e.g. 180 deg) because
# the sensor is mounted rotated about the joint axis relative to the
# convention assumed by the e1-e2 basis.
THETA_OFFSETS = [180.0, 180.0, 180.0]

ser = serial.Serial(PORT, BAUD, timeout=0.1)

# --- Conversion factors (from the MPU-9250 datasheet) -----------
ACCEL_SCALE = 16384.0   # counts per g      (range +/-2 g)
GYRO_SCALE = 131.0      # counts per deg/s  (range +/-250 deg/s)

# --- Madgwick filter parameters ----------------------------------
BETA = 0.1      # accelerometer trust / correction strength
DT = 0.05       # time between samples in seconds (20 Hz)

# one quaternion per limb: body and world frames aligned initially
qs = [[1.0, 0.0, 0.0, 0.0] for _ in range(NUM_LIMBS)]

# --- Gyro bias calibration ---------------------------------------
CAL_SAMPLES = 100          # 5 s at 20 Hz (see Calibration section)

gyro_biases = [[0.0, 0.0, 0.0] for _ in range(NUM_LIMBS)]
cal_sums = [[0.0, 0.0, 0.0] for _ in range(NUM_LIMBS)]
cal_countdown = 0          # samples still to collect (0 = not calibrating)

# --- Data storage for the plot -----------------------------------
theta_data = [[] for _ in range(NUM_LIMBS)]   # theta history per limb (deg)


# --- Small vector helpers (3-vectors as plain lists) --------------

def vec_dot(a, b):
    return a[0] * b[0] + a[1] * b[1] + a[2] * b[2]

def vec_cross(a, b):
    return [a[1] * b[2] - a[2] * b[1],
            a[2] * b[0] - a[0] * b[2],
            a[0] * b[1] - a[1] * b[0]]

def vec_norm(a):
    return math.sqrt(vec_dot(a, a))

def vec_unit(a):
    n = vec_norm(a)
    return [a[0] / n, a[1] / n, a[2] / n] if n > 0 else [0.0, 0.0, 0.0]


# --- Build each limb's e1-e2 plane basis from its manual axis ------
# (eq:plane-basis in the report):
#   e1 = nhat x zhat   (perpendicular to nhat and to z)
#   e2 = nhat x e1     (approximately -zhat for ideal mounting)
# Both are normalised so that p = cos(theta) and s = sin(theta)
# hold exactly even if the mounting angle gamma != 90 deg.

zhat = [0.0, 0.0, 1.0]

nhats, e1s, e2s = [], [], []
for limb, raw_axis in enumerate(NHATS):
    nhat = vec_unit(raw_axis)
    e1_raw = vec_cross(nhat, zhat)
    if vec_norm(e1_raw) < 1e-6:
        # nhat is (anti)parallel to z: the joint rotates about gravity
        # and theta cannot be observed from the gravity vector
        raise SystemExit(f"NHAT of limb {limb + 1} is parallel to z - "
                         "invalid mounting for this method")
    nhats.append(nhat)
    e1s.append(vec_unit(e1_raw))
    e2s.append(vec_unit(vec_cross(nhat, e1s[limb])))


# --- The Madgwick update, per-limb version -------------------------
# Same maths as theta_single_limb.py but operating on the given
# quaternion list in place, so each limb keeps its own state.

def madgwick_update(q, gx, gy, gz, ax, ay, az):
    q0, q1, q2, q3 = q

    # ---- 1) gyro part: qdot = 0.5 * q (x) [0, W] ----
    qdot0 = 0.5 * (-q1 * gx - q2 * gy - q3 * gz)
    qdot1 = 0.5 * ( q0 * gx + q2 * gz - q3 * gy)
    qdot2 = 0.5 * ( q0 * gy - q1 * gz + q3 * gx)
    qdot3 = 0.5 * ( q0 * gz + q1 * gy - q2 * gx)

    # ---- 2) accel part: gradient descent on the error f ----
    norm = math.sqrt(ax * ax + ay * ay + az * az)
    if norm > 0:
        ax, ay, az = ax / norm, ay / norm, az / norm

        f1 = 2 * (q1 * q3 - q0 * q2) - ax
        f2 = 2 * (q0 * q1 + q2 * q3) - ay
        f3 = 2 * (0.5 - q1 * q1 - q2 * q2) - az

        s0 = -2 * q2 * f1 + 2 * q1 * f2
        s1 =  2 * q3 * f1 + 2 * q0 * f2 - 4 * q1 * f3
        s2 = -2 * q0 * f1 + 2 * q3 * f2 - 4 * q2 * f3
        s3 =  2 * q1 * f1 + 2 * q2 * f2

        norm = math.sqrt(s0 * s0 + s1 * s1 + s2 * s2 + s3 * s3)
        if norm > 0:
            qdot0 -= BETA * s0 / norm
            qdot1 -= BETA * s1 / norm
            qdot2 -= BETA * s2 / norm
            qdot3 -= BETA * s3 / norm

    # ---- 3) integrate and renormalise ----
    q0 += qdot0 * DT
    q1 += qdot1 * DT
    q2 += qdot2 * DT
    q3 += qdot3 * DT
    norm = math.sqrt(q0 * q0 + q1 * q1 + q2 * q2 + q3 * q3)
    q[0], q[1], q[2], q[3] = q0 / norm, q1 / norm, q2 / norm, q3 / norm


# --- Joint angle from a limb's quaternion ---------------------------
# Implements the Joint Angle Calculation section:
#   ghat_body = R^T [0 0 1]^T   -> third column of R^T (eq:predicted-gravity)
#   p = ghat_body . e1 = cos(theta)
#   s = ghat_body . e2 = sin(theta)   (eq:projections)
#   theta = atan2(s, p) mod 360       (eq:joint-angle)

def compute_theta(q, e1, e2, offset):
    q0, q1, q2, q3 = q

    g_body = [2 * (q1 * q3 - q0 * q2),
              2 * (q0 * q1 + q2 * q3),
              2 * (0.5 - q1 * q1 - q2 * q2)]

    p = vec_dot(g_body, e1)
    s = vec_dot(g_body, e2)
    theta = (math.degrees(math.atan2(s, p)) + offset) % 360.0
    return theta


# --- Build the window ----------------------------------------------

window = tk.Tk()
window.title("Three Limb Joint Angles (theta)")

fig = Figure(figsize=(8, 5))
theta_plot = fig.add_subplot(1, 1, 1)
theta_plot.set_title("Joint angles")
theta_plot.set_ylabel("theta (deg)")
theta_plot.set_xlabel("Sample number")
theta_plot.set_ylim(0, 360)

theta_lines = []
for limb in range(NUM_LIMBS):
    line, = theta_plot.plot([], [], label=f"theta {limb + 1}")
    theta_lines.append(line)
theta_plot.legend(loc="upper left")
fig.tight_layout()

canvas = FigureCanvasTkAgg(fig, master=window)

# --- Buttons and status text ----------------------------------------
# Packed BEFORE the canvas (anchored to the bottom) so that when the
# window is too small for the full figure, the canvas shrinks instead
# of the buttons being pushed off-screen.

def reset():
    global cal_countdown
    for d in theta_data:
        d.clear()
    for q in qs:
        q[:] = [1.0, 0.0, 0.0, 0.0]
    for bias in gyro_biases:
        bias[:] = [0.0, 0.0, 0.0]
    cal_countdown = 0
    status_cal.config(text="Not calibrated")

def calibrate():
    global cal_countdown
    for limb in range(NUM_LIMBS):
        cal_sums[limb][:] = [0.0, 0.0, 0.0]
        gyro_biases[limb][:] = [0.0, 0.0, 0.0]
    cal_countdown = CAL_SAMPLES
    status_cal.config(text="Calibrating - keep all three sensors still...")

button_row = tk.Frame(window)
button_row.pack(side="bottom", pady=5)

tk.Button(button_row, text="Reset", font=("Arial", 12),
          command=reset).pack(side="left", padx=5)
tk.Button(button_row, text="Calibrate gyros", font=("Arial", 12),
          command=calibrate).pack(side="left", padx=5)

status_cal = tk.Label(window, text="Not calibrated", font=("Arial", 11))
status_cal.pack(side="bottom")
status_axis = tk.Label(
    window,
    text="   ".join(
        f"nhat{limb + 1}: [{n[0]:+.3f}, {n[1]:+.3f}, {n[2]:+.3f}]"
        for limb, n in enumerate(nhats)),
    font=("Arial", 11))
status_axis.pack(side="bottom", pady=(0, 5))

canvas.get_tk_widget().pack(fill="both", expand=True)

# --- Read serial data, run the pipeline, redraw ----------------------

def update():
    global cal_countdown

    while ser.in_waiting:
        line = ser.readline().decode(errors="ignore").strip()
        parts = line.split(",")
        if len(parts) == 6 * NUM_LIMBS:
            try:
                values = [int(p) for p in parts]
            except ValueError:
                continue

            calibrating = cal_countdown > 0
            for limb in range(NUM_LIMBS):
                chunk = values[6 * limb: 6 * limb + 6]
                accel = [chunk[i] / ACCEL_SCALE for i in range(3)]
                gyro = [chunk[i] / GYRO_SCALE for i in range(3, 6)]

                # gyro bias calibration (Calibration section)
                if calibrating:
                    for i in range(3):
                        cal_sums[limb][i] += gyro[i]

                # bias-corrected gyro in deg/s
                gyro_c = [gyro[i] - gyro_biases[limb][i] for i in range(3)]

                # Madgwick update (gyro must be converted to rad/s)
                madgwick_update(qs[limb],
                                math.radians(gyro_c[0]),
                                math.radians(gyro_c[1]),
                                math.radians(gyro_c[2]),
                                accel[0], accel[1], accel[2])

                # joint angle from this limb's orientation and axis
                theta_data[limb].append(
                    compute_theta(qs[limb], e1s[limb], e2s[limb],
                                  THETA_OFFSETS[limb]))

            if calibrating:
                cal_countdown -= 1
                if cal_countdown == 0:
                    for limb in range(NUM_LIMBS):
                        for i in range(3):
                            gyro_biases[limb][i] = (cal_sums[limb][i]
                                                    / CAL_SAMPLES)
                    status_cal.config(
                        text="Calibrated - gyro biases stored for all limbs")

    # redraw
    for limb, line in enumerate(theta_lines):
        d = theta_data[limb]
        line.set_data(range(len(d)), d)
        if d:
            line.set_label(f"theta {limb + 1}: {d[-1]:.1f} deg")
    if any(theta_data):
        theta_plot.legend(loc="upper left")

    theta_plot.relim()
    theta_plot.autoscale_view(scaley=False)  # keep 0-360
    canvas.draw()

    window.after(100, update)

update()
window.mainloop()

ser.close()
