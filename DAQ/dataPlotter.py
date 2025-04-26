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
therm_before_rad_time, therm_before_rad_raw = [], []
therm_after_rad_time, therm_after_rad_raw = [], []
log_file = input("Enter log file name (e.g., test_log.txt): ")
# Read log and collect raw values
with open(log_file, "r") as f:
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
        elif "Temperature before Radiator :" in line:
            match = re.search(r"Temperature before Radiator\s*:\s*([\d\.]+)", line)
            if match:
                value = float(match.group(1))
                therm_before_rad_time.append(t)
                therm_before_rad_raw.append(value)
        elif "Temperature after Radiator :" in line:
            match = re.search(r"Temperature after Radiator\s*:\s*([\d\.]+)", line)
            if match:
                value = float(match.group(1))
                therm_after_rad_time.append(t)
                therm_after_rad_raw.append(value)


# CSV export (EMA only)
def write_csv(filename="parsed_output.csv"):
    data_dict = defaultdict(dict)

    for t, v in zip(mt_time, mt_values_raw):
        if v is not None:
            rounded_t = round(t, 1)
            data_dict[rounded_t]["motor"] = v

    for t, v in zip(bamo_time, bamo_values_raw):
        if v is not None:
            rounded_t = round(t, 1)
            data_dict[rounded_t]["mtrctrl"] = v

    for t, v in zip(volt_time, volt_values):
        if v is not None:
            rounded_t = round(t, 1)
            data_dict[rounded_t]["Volt"] = v

    for t, v in zip(therm_before_rad_time, therm_before_rad_raw):
        if v is not None:
            rounded_t = round(t, 1)
            data_dict[rounded_t]["Therm_Before_Rad"] = v

    for t, v in zip(therm_after_rad_time, therm_after_rad_raw):
        if v is not None:
            rounded_t = round(t, 1)
            data_dict[rounded_t]["Therm_After_Rad"] = v

    all_times = sorted(data_dict.keys())

    with open(filename, "w", newline="") as csvfile:
        writer = csv.writer(csvfile)
        writer.writerow(["Time", "motor", "mtrctrl", "Volt", "Therm_Before_Rad", "Therm_After_Rad"])
        for t in all_times:
            row = data_dict[t]
            writer.writerow([
                t,
                row.get("motor", ""),
                row.get("mtrctrl", ""),
                row.get("Volt", ""),
                row.get("Therm_Before_Rad", ""),
                row.get("Therm_After_Rad", "")
            ])

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


# --- 3. Voltage ---
ax3.plot(volt_time, volt_values, 'o-', color='green', label="Voltage")
ax3.set_title("Voltage Over Time")
ax3.set_xlabel("Time (s)")
ax3.set_ylabel("Voltage (unit)")
ax3.grid(True)
ax3.legend()

plt.tight_layout()
plt.show()
