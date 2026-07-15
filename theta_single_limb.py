# ---------------------------------------------------------------
# theta_single_limb.py
# GUI that computes the joint angle theta of a single robot limb
# from an MPU-9250, following the report pipeline:
#
#   1) gyro bias calibration        (Calibration section)
#   2) Madgwick 6-DOF filter -> q   (Madgwick Filter section)
#   3) theta from gravity in the    (Joint Angle Calculation)
#      e1-e2 plane via atan2
#
# The rotation axis nhat is provided MANUALLY below (NHAT), e.g.
# from a previous run of the axis determination procedure.
#
# Built on raw_data_magw.py. The Arduino sends one line per sample:
#     ax,ay,az,gx,gy,gz
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

# --- Rotation axis (provided manually) ---------------------------
# Unit axis of rotation of the limb in the SENSOR BODY frame,
# e.g. the ideal mounting from the report. It is re-normalised
# below in case the entered values are not exactly unit length.
# Note: flipping the sign of NHAT reverses the direction in which
# theta increases.

NHAT = [-0.707, -0.707, 0.0]     # <-- set your measured axis here

ser = serial.Serial(PORT, BAUD, timeout=0.1)

# --- Conversion factors (from the MPU-9250 datasheet) -----------
ACCEL_SCALE = 16384.0   # counts per g      (range +/-2 g)
GYRO_SCALE = 131.0      # counts per deg/s  (range +/-250 deg/s)

# --- Madgwick filter parameters ----------------------------------
BETA = 0.1      # accelerometer trust / correction strength
DT = 0.05       # time between samples in seconds (20 Hz)

q = [1.0, 0.0, 0.0, 0.0]   # q_initial: body and world frames aligned

# --- Gyro bias calibration ---------------------------------------
CAL_SAMPLES = 100          # 5 s at 20 Hz (see Calibration section)

gyro_bias = [0.0, 0.0, 0.0]
cal_sums = [0.0, 0.0, 0.0]
cal_countdown = 0          # samples still to collect (0 = not calibrating)

# --- Data storage for the plots ----------------------------------
names = ["Accel X", "Accel Y", "Accel Z", "Gyro X", "Gyro Y", "Gyro Z"]
data = [[], [], [], [], [], []]
theta_data = []            # joint angle history (degrees, 0-360)


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


# --- Build the e1-e2 plane basis from the manual axis --------------
# (eq:plane-basis in the report):
#   e1 = nhat x zhat   (perpendicular to nhat and to z)
#   e2 = nhat x e1     (approximately -zhat for ideal mounting)
# Both are normalised so that p = cos(theta) and s = sin(theta)
# hold exactly even if the mounting angle gamma != 90 deg.

nhat = vec_unit(NHAT)

zhat = [0.0, 0.0, 1.0]
e1_raw = vec_cross(nhat, zhat)
if vec_norm(e1_raw) < 1e-6:
    # nhat is (anti)parallel to z: the joint rotates about gravity
    # and theta cannot be observed from the gravity vector
    raise SystemExit("NHAT is parallel to z - invalid mounting for this method")
e1 = vec_unit(e1_raw)
e2 = vec_unit(vec_cross(nhat, e1))

# angles between nhat and each primary axis, for a quick mounting check
# (eq:axis-angles; gamma should be ~90 deg for the intended mounting)
alpha = math.degrees(math.acos(max(-1.0, min(1.0, nhat[0]))))
beta  = math.degrees(math.acos(max(-1.0, min(1.0, nhat[1]))))
gamma = math.degrees(math.acos(max(-1.0, min(1.0, nhat[2]))))


# --- The Madgwick update, unchanged from raw_data_magw.py ---------

def madgwick_update(gx, gy, gz, ax, ay, az):
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


# --- Joint angle from the current quaternion ------------------------
# Implements the Joint Angle Calculation section:
#   ghat_body = R^T [0 0 1]^T   -> third column of R^T (eq:predicted-gravity)
#   p = ghat_body . e1 = cos(theta)
#   s = ghat_body . e2 = sin(theta)   (eq:projections)
#   theta = atan2(s, p) mod 360       (eq:joint-angle)

def compute_theta():
    q0, q1, q2, q3 = q

    g_body = [2 * (q1 * q3 - q0 * q2),
              2 * (q0 * q1 + q2 * q3),
              2 * (0.5 - q1 * q1 - q2 * q2)]

    p = vec_dot(g_body, e1)
    s = vec_dot(g_body, e2)
    theta = math.degrees(math.atan2(s, p)) % 360.0
    return theta


# --- Build the window ----------------------------------------------

window = tk.Tk()
window.title("Single Limb Joint Angle (theta)")

fig = Figure(figsize=(8, 8))
accel_plot = fig.add_subplot(3, 1, 1)
gyro_plot = fig.add_subplot(3, 1, 2)
theta_plot = fig.add_subplot(3, 1, 3)

accel_plot.set_title("Accelerometer")
accel_plot.set_ylabel("g")
gyro_plot.set_title("Gyroscope (bias corrected)")
gyro_plot.set_ylabel("deg/s")
theta_plot.set_title("Joint angle")
theta_plot.set_ylabel("theta (deg)")
theta_plot.set_xlabel("Sample number")
theta_plot.set_ylim(0, 360)

lines = []
for i in range(3):
    line, = accel_plot.plot([], [], label=names[i])
    lines.append(line)
for i in range(3, 6):
    line, = gyro_plot.plot([], [], label=names[i])
    lines.append(line)
theta_line, = theta_plot.plot([], [], label="theta")

accel_plot.legend(loc="upper left")
gyro_plot.legend(loc="upper left")
fig.tight_layout()

canvas = FigureCanvasTkAgg(fig, master=window)
canvas.get_tk_widget().pack(fill="both", expand=True)

# --- Buttons and status text ----------------------------------------

def reset():
    global cal_countdown
    for d in data:
        d.clear()
    theta_data.clear()
    q[:] = [1.0, 0.0, 0.0, 0.0]
    gyro_bias[:] = [0.0, 0.0, 0.0]
    cal_countdown = 0
    status_cal.config(text="Not calibrated")

def calibrate():
    global cal_countdown
    cal_sums[:] = [0.0, 0.0, 0.0]
    gyro_bias[:] = [0.0, 0.0, 0.0]
    cal_countdown = CAL_SAMPLES
    status_cal.config(text="Calibrating - keep the sensor still...")

button_row = tk.Frame(window)
button_row.pack(pady=5)

tk.Button(button_row, text="Reset", font=("Arial", 12),
          command=reset).pack(side="left", padx=5)
tk.Button(button_row, text="Calibrate gyro", font=("Arial", 12),
          command=calibrate).pack(side="left", padx=5)

status_cal = tk.Label(window, text="Not calibrated", font=("Arial", 11))
status_cal.pack()
status_axis = tk.Label(
    window,
    text=f"nhat (manual): [{nhat[0]:+.3f}, {nhat[1]:+.3f}, {nhat[2]:+.3f}]   "
         f"alpha={alpha:.1f}  beta={beta:.1f}  gamma={gamma:.1f} deg",
    font=("Arial", 11))
status_axis.pack(pady=(0, 5))

# --- Read serial data, run the pipeline, redraw ----------------------

def update():
    global cal_countdown

    while ser.in_waiting:
        line = ser.readline().decode(errors="ignore").strip()
        parts = line.split(",")
        if len(parts) == 6:
            try:
                values = [int(p) for p in parts]
            except ValueError:
                continue

            accel = [values[i] / ACCEL_SCALE for i in range(3)]
            gyro = [values[i] / GYRO_SCALE for i in range(3, 6)]

            # gyro bias calibration (Calibration section)
            if cal_countdown > 0:
                for i in range(3):
                    cal_sums[i] += gyro[i]
                cal_countdown -= 1
                if cal_countdown == 0:
                    for i in range(3):
                        gyro_bias[i] = cal_sums[i] / CAL_SAMPLES
                    status_cal.config(text="Calibrated - gyro bias: "
                                           f"{gyro_bias[0]:.2f}, "
                                           f"{gyro_bias[1]:.2f}, "
                                           f"{gyro_bias[2]:.2f} deg/s")

            # bias-corrected gyro in deg/s
            gyro_c = [gyro[i] - gyro_bias[i] for i in range(3)]

            for i in range(3):
                data[i].append(accel[i])
                data[i + 3].append(gyro_c[i])

            # Madgwick update (gyro must be converted to rad/s)
            madgwick_update(math.radians(gyro_c[0]),
                            math.radians(gyro_c[1]),
                            math.radians(gyro_c[2]),
                            accel[0], accel[1], accel[2])

            # joint angle from the current orientation and manual nhat
            theta_data.append(compute_theta())

    # redraw
    x = range(len(data[0]))
    for line, d in zip(lines, data):
        line.set_data(x, d)
    theta_line.set_data(range(len(theta_data)), theta_data)

    for line, name, d in zip(lines, names, data):
        if d:
            line.set_label(f"{name}: {d[-1]:.2f}")
    accel_plot.legend(loc="upper left")
    gyro_plot.legend(loc="upper left")
    if theta_data:
        theta_line.set_label(f"theta: {theta_data[-1]:.1f} deg")
        theta_plot.legend(loc="upper left")

    accel_plot.relim(); accel_plot.autoscale_view()
    gyro_plot.relim(); gyro_plot.autoscale_view()
    theta_plot.relim(); theta_plot.autoscale_view(scaley=False)  # keep 0-360
    canvas.draw()

    window.after(100, update)

update()
window.mainloop()

ser.close()