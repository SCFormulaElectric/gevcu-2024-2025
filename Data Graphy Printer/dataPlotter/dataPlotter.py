import re
import matplotlib.pyplot as plt

# Containers
volt_time, volt_values = [], []
bamo_time, bamo_values = [], []
mt_time, mt_values = [], []

# Read from test_log.txt or actual log
with open("test_log.txt", "r") as f:
    for line in f:
        # Extract timestamp
        time_match = re.search(r'I\(([\d\.]+)\)', line)
        if not time_match:
            continue
        t = float(time_match.group(1))

        # Voltage log
        if "Voltage reading :" in line:
            value = int(re.search(r"Voltage reading\s*:\s*(\d+)", line).group(1))
            volt_time.append(t)
            volt_values.append(value)

        # BAMOCAR temp log
        elif "BAMOCAR temp :" in line:
            value = float(re.search(r"BAMOCAR temp\s*:\s*([\d\.]+)", line).group(1))
            bamo_time.append(t)
            bamo_values.append(value)

        # Motor temp log
        elif "motor temp :" in line:
            value = int(re.search(r"motor temp\s*:\s*(\d+)", line).group(1))
            mt_time.append(t)
            mt_values.append(value)

# === Plot Voltage ===
plt.figure(figsize=(10, 4))
plt.plot(volt_time, volt_values, 'o-', color='green', label="Voltage")
plt.title("Voltage Over Time")
plt.xlabel("Time (s)")
plt.ylabel("Voltage (unit)")
plt.grid(True)
plt.legend()
plt.tight_layout()
plt.show()

# === Plot BAMOCAR Temp ===
plt.figure(figsize=(10, 4))
plt.plot(bamo_time, bamo_values, 's-', color='red', label="BAMOCAR Temp")
plt.title("BAMOCAR Temperature Over Time")
plt.xlabel("Time (s)")
plt.ylabel("Temperature (°C)")
plt.grid(True)
plt.legend()
plt.tight_layout()
plt.show()

# === Plot Motor Temp ===
plt.figure(figsize=(10, 4))
plt.plot(mt_time, mt_values, '^-', color='orange', label="Motor Temp")
plt.title("Motor Temperature Over Time")
plt.xlabel("Time (s)")
plt.ylabel("Temperature (°C)")
plt.grid(True)
plt.legend()
plt.tight_layout()
plt.show()
