import re
import matplotlib.pyplot as plt
import csv
from collections import defaultdict

# EMA parameters
ALPHA = 0.3
MIN_WINDOW = 10

# Containers
volt_time, volt_values = [], []
bamo_time, bamo_values_raw = [], []
mt_time, mt_values_raw = [], []

# Read log and collect raw values
with open("test_log.txt", "r") as f:
    for line in f:
        time_match = re.search(r'I\(([\d\.]+)\)', line)
        if not time_match:
            continue
        t = float(time_match.group(1))

        if "Voltage reading :" in line:
            value = int(re.search(r"Voltage reading\s*:\s*(\d+)", line).group(1))
            volt_time.append(t)
            volt_values.append(value)
        elif "BAMOCAR temp :" in line:
            value = float(re.search(r"BAMOCAR temp\s*:\s*([\d\.]+)", line).group(1))
            bamo_time.append(t)
            bamo_values_raw.append(value)
        elif "motor temp :" in line:
            value = int(re.search(r"motor temp\s*:\s*(\d+)", line).group(1))
            mt_time.append(t)
            mt_values_raw.append(value)

# EMA function
def apply_ema(times, values):
    smoothed = []
    ema = None
    for i in range(len(values)):
        if i < MIN_WINDOW:
            smoothed.append(None)
            continue
        if ema is None:
            ema = sum(values[i - MIN_WINDOW:i]) / MIN_WINDOW
        else:
            ema = ALPHA * values[i] + (1 - ALPHA) * ema
        smoothed.append(ema)
    return smoothed

# Apply EMA
mt_values_ema = apply_ema(mt_time, mt_values_raw)
bamo_values_ema = apply_ema(bamo_time, bamo_values_raw)

# CSV export (EMA only)
def write_csv(filename="parsed_output.csv"):
    data_dict = defaultdict(dict)
    for t, v in zip(mt_time, mt_values_ema):
        if v is not None:
            data_dict[t]["MT"] = v
    for t, v in zip(bamo_time, bamo_values_ema):
        if v is not None:
            data_dict[t]["MCT"] = v
    for t, v in zip(volt_time, volt_values):
        data_dict[t]["Volt"] = v

    all_times = sorted(data_dict.keys())
    with open(filename, "w", newline="") as csvfile:
        writer = csv.writer(csvfile)
        writer.writerow(["Time", "MT (EMA)", "MCT (EMA)", "Volt"])
        for t in all_times:
            mt = data_dict[t].get("MT", "")
            mct = data_dict[t].get("MCT", "")
            volt = data_dict[t].get("Volt", "")
            writer.writerow([t, mt, mct, volt])

write_csv()

# === PLOT: 3 vertically stacked graphs ===
fig, (ax1, ax2, ax3) = plt.subplots(3, 1, figsize=(12, 12), sharex=True)

# --- 1. Raw Temps ---
ax1.plot(mt_time, mt_values_raw, '^-', color='orange', label="Motor Temp (Raw)")
ax1.plot(bamo_time, bamo_values_raw, 's-', color='red', label="BAMOCAR Temp (Raw)")
ax1.set_title("Raw Temperatures")
ax1.set_ylabel("Temperature (°C)")
ax1.grid(True)
ax1.legend()

# --- 2. EMA Temps ---
mt_plot_time = [t for t, v in zip(mt_time, mt_values_ema) if v is not None]
mt_plot_val = [v for v in mt_values_ema if v is not None]
bamo_plot_time = [t for t, v in zip(bamo_time, bamo_values_ema) if v is not None]
bamo_plot_val = [v for v in bamo_values_ema if v is not None]

ax2.plot(mt_plot_time, mt_plot_val, '^-', color='darkorange', label="Motor Temp (EMA)")
ax2.plot(bamo_plot_time, bamo_plot_val, 's-', color='darkred', label="BAMOCAR Temp (EMA)")
ax2.set_title("EMA-Smoothed Temperatures")
ax2.set_ylabel("Temperature (°C)")
ax2.grid(True)
ax2.legend()

# --- 3. Voltage ---
ax3.plot(volt_time, volt_values, 'o-', color='green', label="Voltage")
ax3.set_title("Voltage Over Time")
ax3.set_xlabel("Time (s)")
ax3.set_ylabel("Voltage (unit)")
ax3.grid(True)
ax3.legend()

plt.tight_layout()
plt.show()
