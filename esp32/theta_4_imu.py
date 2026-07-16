# ---------------------------------------------------------------
# theta_4_imu.py
# GUI for the ESP32 + TCA9548A + 4x MPU-9250 setup
# (esp32/raw_data_4_imu/raw_data_4_imu.ino).
#
# IMUs 1-3 (mux channels 0-2): joint angle theta of each limb,
#   same pipeline as theta_3_limb.py:
#     1) gyro bias calibration        (Calibration section)
#     2) Madgwick 6-DOF filter -> q   (Madgwick Filter section)
#     3) theta from gravity in the    (Joint Angle Calculation)
#        e1-e2 plane via atan2
#
# IMU 4 (mux channel 3): orientation as intrinsic ZYX Euler angles
#   (yaw about Z, then pitch about Y', then roll about X''),
#   taken from the same Madgwick quaternion. Without a magnetometer
#   the yaw is relative to the start-up heading and drifts slowly.
#
# Calibration is AUTOMATIC: see the marked block below.
#
# The ESP32 sends one line per sample with 24 values:
#     ax1,ay1,az1,gx1,gy1,gz1, ... ,ax4,ay4,az4,gx4,gy4,gz4
#
# Requirements:  pip install pyserial matplotlib
# Change PORT below to match your ESP32.
# ---------------------------------------------------------------

import tkinter as tk
import math
import serial
from matplotlib.figure import Figure
from matplotlib.backends.backend_tkagg import FigureCanvasTkAgg

PORT = "COM6"     # <-- change this to your ESP32's port
BAUD = 115200      # must match Serial.begin() in the sketch

NUM_IMUS = 4       # total sensors on the mux
NUM_LIMBS = 3      # IMUs 1-3 are limb joints; IMU 4 is euler angles

# --- Rotation axes of the limbs (provided manually) ---------------
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
THETA_OFFSETS = [180.0, 180.0, 180.0]

# --- Conversion factors (from the MPU-9250 datasheet) -----------
ACCEL_SCALE = 16384.0   # counts per g      (range +/-2 g)
GYRO_SCALE = 131.0      # counts per deg/s  (range +/-250 deg/s)

DT = 0.05       # time between samples in seconds (20 Hz)

# =================================================================
# AUTO-CALIBRATION AND HIGH-GAIN START-UP
#
# The pipeline starts itself in three phases, no button needed:
#
#   Phase 1  CALIBRATING (first CAL_SAMPLES samples, ~5 s)
#            All sensors must be STILL. The gyro readings are
#            averaged to get each IMU's bias. The filters do not
#            run yet.
#
#   Phase 2  CONVERGING (next HIGH_GAIN_SAMPLES samples, ~5 s)
#            The Madgwick gain beta is set HIGH so the quaternions
#            snap quickly from their arbitrary initial value onto
#            the true orientation given by the accelerometers.
#            Angles shown during this phase are still settling.
#
#   Phase 3  RUNNING (from then on)
#            beta drops to its normal low value: the gyros do the
#            work and the accelerometers apply gentle corrections.
#
# The Restart button returns to Phase 1 (e.g. after moving a sensor).
# =================================================================
CAL_SAMPLES = 100          # Phase 1 length: 5 s at 20 Hz
HIGH_GAIN_SAMPLES = 100    # Phase 2 length: 5 s at 20 Hz
BETA_HIGH = 2.5            # Phase 2 gain: fast initial convergence
BETA_NORMAL = 0.1          # Phase 3 gain: normal smooth tracking

PHASE_CALIBRATING, PHASE_CONVERGING, PHASE_RUNNING = range(3)
phase = PHASE_CALIBRATING
phase_samples = 0          # samples consumed in the current phase
beta = BETA_HIGH           # current Madgwick gain
# =================================================================

ser = serial.Serial(PORT, BAUD, timeout=0.1)

# one quaternion per IMU: body and world frames aligned initially
qs = [[1.0, 0.0, 0.0, 0.0] for _ in range(NUM_IMUS)]

gyro_biases = [[0.0, 0.0, 0.0] for _ in range(NUM_IMUS)]
cal_sums = [[0.0, 0.0, 0.0] for _ in range(NUM_IMUS)]

# --- Data storage for the plots -----------------------------------
theta_data = [[] for _ in range(NUM_LIMBS)]   # theta history per limb (deg)
euler_data = [[], [], []]                     # IMU 4 yaw, pitch, roll (deg)
euler_names = ["Yaw (Z)", "Pitch (Y')", "Roll (X'')"]


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

zhat = [0.0, 0.0, 1.0]

nhats, e1s, e2s = [], [], []
for limb, raw_axis in enumerate(NHATS):
    nhat = vec_unit(raw_axis)
    e1_raw = vec_cross(nhat, zhat)
    if vec_norm(e1_raw) < 1e-6:
        raise SystemExit(f"NHAT of limb {limb + 1} is parallel to z - "
                         "invalid mounting for this method")
    nhats.append(nhat)
    e1s.append(vec_unit(e1_raw))
    e2s.append(vec_unit(vec_cross(nhat, e1s[limb])))


# --- The Madgwick update, per-IMU version --------------------------
# Same maths as theta_single_limb.py but operating on the given
# quaternion list in place, with the gain taken from the current
# start-up phase (see AUTO-CALIBRATION block above).

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
            qdot0 -= beta * s0 / norm
            qdot1 -= beta * s1 / norm
            qdot2 -= beta * s2 / norm
            qdot3 -= beta * s3 / norm

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
#   theta = atan2(s, p) + offset, mod 360

def compute_theta(q, e1, e2, offset):
    q0, q1, q2, q3 = q

    g_body = [2 * (q1 * q3 - q0 * q2),
              2 * (q0 * q1 + q2 * q3),
              2 * (0.5 - q1 * q1 - q2 * q2)]

    p = vec_dot(g_body, e1)
    s = vec_dot(g_body, e2)
    theta = (math.degrees(math.atan2(s, p)) + offset) % 360.0
    return theta


# --- Intrinsic ZYX Euler angles from IMU 4's quaternion --------------
# Standard aerospace sequence: yaw about Z, then pitch about the new
# Y', then roll about the newest X''. Returned in degrees:
#   yaw, roll in (-180, 180], pitch in [-90, 90].

def compute_euler_zyx(q):
    q0, q1, q2, q3 = q

    yaw = math.atan2(2 * (q0 * q3 + q1 * q2),
                     1 - 2 * (q2 * q2 + q3 * q3))
    sinp = max(-1.0, min(1.0, 2 * (q0 * q2 - q3 * q1)))
    pitch = math.asin(sinp)
    roll = math.atan2(2 * (q0 * q1 + q2 * q3),
                      1 - 2 * (q1 * q1 + q2 * q2))

    return math.degrees(yaw), math.degrees(pitch), math.degrees(roll)


# --- Build the window ----------------------------------------------

window = tk.Tk()
window.title("Limb Joint Angles + IMU 4 Euler Angles (ESP32)")

fig = Figure(figsize=(8, 7))
theta_plot = fig.add_subplot(2, 1, 1)
euler_plot = fig.add_subplot(2, 1, 2)

theta_plot.set_title("Limb joint angles (IMUs 1-3)")
theta_plot.set_ylabel("theta (deg)")
theta_plot.set_ylim(0, 360)

euler_plot.set_title("IMU 4 orientation (intrinsic ZYX Euler angles)")
euler_plot.set_ylabel("angle (deg)")
euler_plot.set_xlabel("Sample number")
euler_plot.set_ylim(-180, 180)

theta_lines = []
for limb in range(NUM_LIMBS):
    line, = theta_plot.plot([], [], label=f"theta {limb + 1}")
    theta_lines.append(line)
euler_lines = []
for name in euler_names:
    line, = euler_plot.plot([], [], label=name)
    euler_lines.append(line)

theta_plot.legend(loc="upper left")
euler_plot.legend(loc="upper left")
fig.tight_layout()

canvas = FigureCanvasTkAgg(fig, master=window)

# --- Controls and status text ----------------------------------------
# Packed BEFORE the canvas (anchored to the bottom) so that when the
# window is too small for the full figure, the canvas shrinks instead
# of the controls being pushed off-screen.

def restart():
    """Back to Phase 1 of the auto-calibration sequence."""
    global phase, phase_samples, beta
    for d in theta_data:
        d.clear()
    for d in euler_data:
        d.clear()
    for q in qs:
        q[:] = [1.0, 0.0, 0.0, 0.0]
    for imu in range(NUM_IMUS):
        gyro_biases[imu][:] = [0.0, 0.0, 0.0]
        cal_sums[imu][:] = [0.0, 0.0, 0.0]
    phase = PHASE_CALIBRATING
    phase_samples = 0
    beta = BETA_HIGH
    status_phase.config(text="Phase 1/3: calibrating - keep all "
                             "sensors still...", fg="darkorange")

button_row = tk.Frame(window)
button_row.pack(side="bottom", pady=5)

tk.Button(button_row, text="Restart (auto-calibrate)", font=("Arial", 12),
          command=restart).pack(side="left", padx=5)

status_phase = tk.Label(window, font=("Arial", 11),
                        text="Phase 1/3: calibrating - keep all "
                             "sensors still...", fg="darkorange")
status_phase.pack(side="bottom")
status_axis = tk.Label(
    window,
    text="   ".join(
        f"nhat{limb + 1}: [{n[0]:+.3f}, {n[1]:+.3f}, {n[2]:+.3f}]"
        for limb, n in enumerate(nhats)),
    font=("Arial", 11))
status_axis.pack(side="bottom", pady=(0, 5))

canvas.get_tk_widget().pack(fill="both", expand=True)

# --- Read serial data, run the pipeline, redraw ----------------------

def process_sample(values):
    """One 24-value sample through the phased pipeline (see the
    AUTO-CALIBRATION AND HIGH-GAIN START-UP block above)."""
    global phase, phase_samples, beta

    accels, gyros = [], []
    for imu in range(NUM_IMUS):
        chunk = values[6 * imu: 6 * imu + 6]
        accels.append([chunk[i] / ACCEL_SCALE for i in range(3)])
        gyros.append([chunk[i] / GYRO_SCALE for i in range(3, 6)])

    # Phase 1: accumulate gyro readings for the bias, filters idle
    if phase == PHASE_CALIBRATING:
        for imu in range(NUM_IMUS):
            for i in range(3):
                cal_sums[imu][i] += gyros[imu][i]
        phase_samples += 1
        if phase_samples >= CAL_SAMPLES:
            for imu in range(NUM_IMUS):
                for i in range(3):
                    gyro_biases[imu][i] = cal_sums[imu][i] / CAL_SAMPLES
            phase = PHASE_CONVERGING
            phase_samples = 0
            beta = BETA_HIGH
            status_phase.config(text="Phase 2/3: converging "
                                     "(high gain)...", fg="darkorange")
        return

    # Phase 2 -> 3: drop to the normal gain once converged
    if phase == PHASE_CONVERGING:
        phase_samples += 1
        if phase_samples >= HIGH_GAIN_SAMPLES:
            phase = PHASE_RUNNING
            beta = BETA_NORMAL
            status_phase.config(text="Phase 3/3: running", fg="darkgreen")

    # Phases 2 and 3: run every filter and record the angles
    for imu in range(NUM_IMUS):
        gyro_c = [gyros[imu][i] - gyro_biases[imu][i] for i in range(3)]
        madgwick_update(qs[imu],
                        math.radians(gyro_c[0]),
                        math.radians(gyro_c[1]),
                        math.radians(gyro_c[2]),
                        accels[imu][0], accels[imu][1], accels[imu][2])

    for limb in range(NUM_LIMBS):
        theta_data[limb].append(
            compute_theta(qs[limb], e1s[limb], e2s[limb],
                          THETA_OFFSETS[limb]))

    yaw, pitch, roll = compute_euler_zyx(qs[3])
    euler_data[0].append(yaw)
    euler_data[1].append(pitch)
    euler_data[2].append(roll)


def update():
    while ser.in_waiting:
        line = ser.readline().decode(errors="ignore").strip()
        parts = line.split(",")
        if len(parts) == 6 * NUM_IMUS:
            try:
                values = [int(p) for p in parts]
            except ValueError:
                continue
            process_sample(values)

    # redraw
    for limb, line in enumerate(theta_lines):
        d = theta_data[limb]
        line.set_data(range(len(d)), d)
        if d:
            line.set_label(f"theta {limb + 1}: {d[-1]:.1f} deg")
    for i, line in enumerate(euler_lines):
        d = euler_data[i]
        line.set_data(range(len(d)), d)
        if d:
            line.set_label(f"{euler_names[i]}: {d[-1]:.1f} deg")

    if any(theta_data):
        theta_plot.legend(loc="upper left")
        euler_plot.legend(loc="upper left")

    theta_plot.relim()
    theta_plot.autoscale_view(scaley=False)  # keep 0-360
    euler_plot.relim()
    euler_plot.autoscale_view(scaley=False)  # keep -180..180
    canvas.draw()

    window.after(100, update)

update()
window.mainloop()

ser.close()
