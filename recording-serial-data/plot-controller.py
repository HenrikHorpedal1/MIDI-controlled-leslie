import csv
import matplotlib.pyplot as plt

# ------------------------------------------------------
# 1) Set your log file path here
# ------------------------------------------------------
FILE_PATH = "recoded-controller-shite.csv"
# e.g. FILE_PATH = "/Volumes/Leslie-SD/log.csv"
# ------------------------------------------------------


time_s = []
meas_rpm = []
ref_rpm = []
P_vals = []
I_vals = []
u_vals = []
pwm_vals = []

with open(FILE_PATH, "r") as f:
    reader = csv.reader(f)
    header = next(reader, None)  # try to read header

    for row in reader:
        if not row or len(row) < 7:
            continue

        try:
            millis = int(row[0])
            measuredRPMx100 = float(row[1])
            targetRPMx100   = float(row[2])
            P               = float(row[3])
            I               = float(row[4])
            u               = float(row[5])
            pwm             = float(row[6])
        except ValueError:
            # skip malformed lines
            continue

        time_s.append(millis / 1000.0)
        meas_rpm.append(measuredRPMx100 / 100.0)
        ref_rpm.append(targetRPMx100 / 100.0)
        P_vals.append(P)
        I_vals.append(I)
        u_vals.append(u)
        pwm_vals.append(pwm)


if not time_s:
    print("No valid data found in file:", FILE_PATH)
    raise SystemExit

# ------------------------------------------------------
# Plotting
# ------------------------------------------------------
fig, axs = plt.subplots(3, 1, figsize=(10, 8), sharex=True)

# 1) RPMs
axs[0].plot(time_s, meas_rpm, label="Measured RPM")
axs[0].plot(time_s, ref_rpm,  label="Target RPM", linestyle="--")
axs[0].set_ylabel("RPM")
axs[0].set_title("Speed Controller Log")
axs[0].grid(True)
axs[0].legend()

# 2) Control terms
axs[1].plot(time_s, P_vals, label="P")
axs[1].plot(time_s, I_vals, label="I")
axs[1].plot(time_s, u_vals, label="u (P+I)")
axs[1].set_ylabel("Control terms")
axs[1].grid(True)
axs[1].legend()

# 3) PWM command
axs[2].plot(time_s, pwm_vals, label="PWM command")
axs[2].set_xlabel("Time [s]")
axs[2].set_ylabel("PWM")
axs[2].grid(True)
axs[2].legend()

plt.tight_layout()
plt.show()
