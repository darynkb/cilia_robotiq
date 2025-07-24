import pandas as pd
import matplotlib.pyplot as plt

# Load the CSV file
df = pd.read_csv("/home/crisp_laptop/robotiq_ws/src/sensor_pkg/sensor_pkg/recorded_data/smooth/sensor1_data_20250710_150913.csv", parse_dates=['timestamp'])

# Reshape data into per-sensor time series
sensors = ['1', '2', '3', '4']
axes = ['x', 'y', 'z']

# Plot each sensor in its own subplot
fig, axs = plt.subplots(4, 1, figsize=(12, 10), sharex=True)
fig.suptitle("Sensor Readings Over Time")

for i, sensor in enumerate(sensors):
    for axis in axes:
        col = f"{axis}{sensor}"
        axs[i].plot(df['timestamp'], df[col], label=f"{axis.upper()}{sensor}")
    axs[i].set_ylabel(f"Sensor {sensor}")
    axs[i].legend(loc='upper right')
    axs[i].grid(True)

axs[-1].set_xlabel("Timestamp")
plt.tight_layout()
plt.subplots_adjust(top=0.93)
plt.show()
