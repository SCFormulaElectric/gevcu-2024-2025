import re
import matplotlib.pyplot as plt
import csv
from collections import defaultdict

# Containers
volt_time, volt_values = [], []
bamo_time, bamo_values = [], []
mt_time, mt_values = [], []

# Parse the log file
with open("test_log.txt", "r") as f:
    for line in f:
        # Extract timestamp
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
            bamo_values.append(value)
        elif "motor temp :" in line:
            value = int(re.search(r"motor temp\s*:\s*(\d+)", line).group(1))
            mt_time.append(t)
            mt_values.append(value)

# Function to write aligned data to CSV
def write_csv(filename="parsed_output.csv"):
    data_dict = defaultdict(dict)

    for t, v in zip(mt_time, mt_values):
        data_dict[t]["MT"] = v
    for t, v in zip(bamo_time, bamo_values):
        data_dict[t]["MCT"] = v
    for t, v in zip(volt_time, volt_values):
        data_dict[t]["Volt"] = v

    # Sort by time
    all_times = sorted(data_dict.keys())

    with open(filename, "w", newline="") as csvfile:
        writer = csv.writer(csvfile)
        writer.writerow(["Time", "MT", "MCT", "Volt"])
        for t in all_times:
            mt = data_dict[t].get("MT", "")
            mct = data_dict[t].get("MCT", "")
            volt = data_dict[t].get("Volt", "")
            writer.writerow([t, mt, mct, volt])

# Call the function to export data
write_csv()

# Plotting code (unchanged)
fig, (ax1, ax2) = plt.subplots(2, 1, figsize=(100, 8), sharex=True) # change here to make graph change in size

ax1.plot(bamo_time, bamo_values, 's-', color='red', label="BAMOCAR Temp")
ax1.plot(mt_time, mt_values, '^-', color='orange', label="Motor Temp")
ax1.set_title("Temperature Over Time")
ax1.set_ylabel("Temperature (°C)")
ax1.grid(True)
ax1.legend()

ax2.plot(volt_time, volt_values, 'o-', color='green', label="Voltage")
ax2.set_title("Voltage Over Time")
ax2.set_xlabel("Time (s)")
ax2.set_ylabel("Voltage (unit)")
ax2.grid(True)
ax2.legend()

plt.tight_layout()
plt.show()
