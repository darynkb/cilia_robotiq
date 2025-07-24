import pandas as pd
import matplotlib.pyplot as plt

# Load the CSV file
df = pd.read_csv("/home/crisp_laptop/robotiq_ws/src/sensor_pkg/sensor_pkg/recorded_data/no_object/sensor0_data_20250721_183034.csv", parse_dates=['timestamp'])

# Sensor and axis labels
sensors = ['1', '2', '3', '4']
axes = ['x', 'y', 'z']

# Create subplots: one for each sensor
fig, axs = plt.subplots(4, 1, figsize=(14, 12), sharex=True)
fig.suptitle("Raw vs Calibrated Sensor Readings Over Time")

# Plot for each sensor
for i, sensor in enumerate(sensors):
    for axis in axes:
        raw_col = f"{axis}{sensor}"
        calib_col = f"{axis}{sensor}_calib"

        if raw_col in df.columns and calib_col in df.columns:
            axs[i].plot(df['timestamp'], df[raw_col], label=f"{axis.upper()}{sensor} Raw", linestyle='--')
            axs[i].plot(df['timestamp'], df[calib_col], label=f"{axis.upper()}{sensor} Calib", linewidth=1.5)
    
    axs[i].set_ylabel(f"Sensor {sensor}")
    axs[i].legend(loc='upper right')
    axs[i].grid(True)

axs[-1].set_xlabel("Timestamp")
plt.tight_layout()
plt.subplots_adjust(top=0.93)
plt.show()