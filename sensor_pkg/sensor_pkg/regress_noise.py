import os
import pandas as pd
import numpy as np
import matplotlib.pyplot as plt
from scipy.signal import resample
from sklearn.preprocessing import PolynomialFeatures
# from sklearn.linear_model import LinearRegression
from sklearn.linear_model import RANSACRegressor, LinearRegression

# --- CONFIGURATION ---
directory = '/home/crisp_laptop/robotiq_ws/src/sensor_pkg/sensor_pkg/recorded_data/no_object/'  # update this to your actual folder path
file_ext = '.csv'
target_length = 100
poly_order = 2       # Degree of polynomial regression

# --- STEP 1: FIND ALL CSV FILES ---
csv_files = [f for f in os.listdir(directory) if f.startswith('sensor0_') and f.endswith(file_ext)]
if not csv_files:
    raise FileNotFoundError("No CSV files found in the directory.")
print(f"Found {len(csv_files)} CSV files.")

# --- STEP 2: COLLECT DATA ACROSS FILES ---
data = {'x': {}, 'y': {}, 'z': {}}

for file in csv_files:
    csv_path = os.path.join(directory, file)
    df = pd.read_csv(csv_path)

    calib_cols = [col for col in df.columns if col.endswith('_calib')]

    for col in calib_cols:
        axis = col[0]
        taxel_id = col[1:-6]  # From 'x2_calib' extract '2'

        if axis not in ['x', 'y', 'z']:
            continue

        if taxel_id not in data[axis]:
            data[axis][taxel_id] = []

        resampled = resample(df[col].values, target_length)
        data[axis][taxel_id].append(resampled)

print("Resampling and data collection complete.")

# --- STEP 3: POLYNOMIAL REGRESSION ---
regression_results = {}

for axis in ['x', 'y', 'z']:
    for taxel_id, trials in data[axis].items():
        stacked = np.stack(trials, axis=0)  # Shape: (n_trials, target_length)
        mean_signal = np.mean(stacked, axis=0)

        X = np.arange(target_length).reshape(-1, 1)
        y = mean_signal.reshape(-1, 1)

        # Polynomial regression
        poly = PolynomialFeatures(degree=poly_order)
        X_poly = poly.fit_transform(X)

        model = LinearRegression()
        model.fit(X_poly, y)

        y_pred = model.predict(X_poly)

        regression_results[f"{axis}{taxel_id}"] = {
            'model': model,
            'poly': poly,
            'slope': model.coef_,
            'intercept': model.intercept_,
            'X': X,
            'y_true': y,
            'y_pred': y_pred
        }

print("Polynomial regression complete.")

# --- STEP 4: PLOTTING SAMPLE RESULTS ---
# import random
# sample_keys = random.sample(list(regression_results.keys()), min(6, len(regression_results)))
sample_keys = list(regression_results.keys())

import math
num_plots = len(sample_keys)
cols = 4
rows = math.ceil(num_plots / cols)

plt.figure(figsize=(cols * 5, rows * 3))
for i, key in enumerate(sample_keys, 1):
    result = regression_results[key]
    plt.subplot(rows, cols, i)
    plt.plot(result['X'], result['y_true'], label='Mean Signal')
    plt.plot(result['X'], result['y_pred'], '--', label='Poly Regression')
    plt.title(f"{key} (2nd Order)")
    plt.xlabel("Time Step")
    plt.ylabel("Force")
    plt.legend()

plt.tight_layout()
plt.show()



# # --- STEP 1: Find relevant CSVs ---
# csv_files = [f for f in os.listdir(directory) if f.startswith('sensor0_') and f.endswith('.csv')]
# if not csv_files:
#     raise FileNotFoundError("No sensor0_*.csv files found.")

# print(f"Found {len(csv_files)} CSV files.")

# # --- STEP 2: Read and resample calibrated data ---
# data = {'x': {}, 'y': {}, 'z': {}}

# for file in csv_files:
#     path = os.path.join(directory, file)
#     df = pd.read_csv(path)

#     calib_cols = [col for col in df.columns if col.endswith('_calib')]

#     for col in calib_cols:
#         axis = col[0]                         # 'x', 'y', or 'z'
#         taxel_id = col[1:-6]                  # from 'x1_calib' → '1'

#         if axis not in ['x', 'y', 'z']:
#             continue

#         if taxel_id not in data[axis]:
#             data[axis][taxel_id] = []

#         resampled = resample(df[col].values, target_length)
#         data[axis][taxel_id].append(resampled)

# print("Data resampled.")

# # --- STEP 3: RANSAC Regression ---
# regression_results = {}

# for axis in ['x', 'y', 'z']:
#     for taxel_id, trials in data[axis].items():
#         stacked = np.stack(trials, axis=0)     # Shape: (n_trials, target_length)
#         mean_signal = np.mean(stacked, axis=0)

#         X = np.arange(target_length).reshape(-1, 1)
#         y = mean_signal.reshape(-1, 1)

#         model = RANSACRegressor(estimator=LinearRegression(), min_samples=0.7)
#         model.fit(X, y)
#         y_pred = model.predict(X)

#         regression_results[f"{axis}{taxel_id}"] = {
#             'model': model,
#             'X': X,
#             'y_true': y,
#             'y_pred': y_pred
#         }

# print("RANSAC regression complete.")

# # --- STEP 4: Plot sample regressions ---
# import random
# sample_keys = random.sample(list(regression_results.keys()), min(6, len(regression_results)))

# plt.figure(figsize=(14, 8))
# for i, key in enumerate(sample_keys, 1):
#     result = regression_results[key]
#     plt.subplot(2, 3, i)
#     plt.plot(result['X'], result['y_true'], label='Mean Signal')
#     plt.plot(result['X'], result['y_pred'], '--', label='RANSAC Fit')
#     plt.title(f"{key} (RANSAC)")
#     plt.xlabel("Time Step")
#     plt.ylabel("Force")
#     plt.legend()

# plt.tight_layout()
# plt.show()