import matplotlib.pyplot as plt

# ------------------------------------------------------
# Replace this with your actual file path
FILE_PATH = "/Volumes/NO NAME/log.csv"
# ------------------------------------------------------
time = []
value = []

with open(FILE_PATH, "r") as f:
    for line in f:
        line = line.strip()
        if not line:
            continue

        if "," not in line:
            continue

        t, v = line.split(",")

        # Skip header
        if t.lower() == "millis":
            continue

        try:
            time.append(int(t))
            value.append(int(v))
        except ValueError:
            # Skip bad lines
            continue

plt.figure(figsize=(12, 4))
plt.step(time, value, where="post")
plt.xlabel("Time (ms)")
plt.ylabel("Pin Value")
plt.title("Digital Pin Change Log")
plt.ylim(-0.2, 1.2)
plt.grid(True)
plt.tight_layout()
plt.show()
