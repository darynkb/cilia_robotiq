import pandas as pd
import matplotlib.pyplot as plt

# Load the CSV file
df = pd.read_csv("/home/crisp_laptop/robotiq_ws/src/sensor_pkg/sensor_pkg/recorded_data/gripper_positions_trial_2.csv")  # Replace with your actual filename

# Convert microseconds to seconds for easier plotting
df["time_s"] = (df["timestamp_us"] - df["timestamp_us"].iloc[0]) / 1_000_000

# Plot position over time
plt.figure(figsize=(10, 4))
plt.plot(df["time_s"], df["position"], marker='o', linestyle='-')
plt.xlabel("Time (seconds)")
plt.ylabel("Gripper Position")
plt.title("Gripper Position Over Time")
plt.grid(True)
plt.tight_layout()
plt.show()
