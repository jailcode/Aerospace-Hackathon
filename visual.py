from matplotlib import pyplot as mpl
import pandas as pd

data = pd.read_csv("results.csv")

fig, (ax1, ax2, ax3) = mpl.subplots(3, 1, figsize=(10, 15))

# ------------------------------
# AXIS 0
# ------------------------------
axis_0 = data[data['axis'] == 0]
ax1.plot(axis_0['iteration'], axis_0['gyro_measurement'], label='Gyro Measurement')
ax1.plot(axis_0['iteration'], axis_0['rate_command'], label='Rate Command')
ax1.plot(axis_0['iteration'], axis_0['gimbal_axis'], label='Gimbal Axis')
ax1.plot(axis_0['iteration'], axis_0['server_motorized'], label='Servo')
ax1.set_title("Axis 0 - Raw Data")
ax1.set_xlabel("Iteration")
ax1.set_ylabel("Raw Value")
ax1.legend()

# ------------------------------
# AXIS 1
# ------------------------------
axis_1 = data[data['axis'] == 1]
ax2.plot(axis_1['iteration'], axis_1['gyro_measurement'], label='Gyro Measurement')
ax2.plot(axis_1['iteration'], axis_1['rate_command'], label='Rate Command')
ax2.plot(axis_1['iteration'], axis_1['gimbal_axis'], label='Gimbal Axis')
ax2.plot(axis_1['iteration'], axis_1['server_motorized'], label='Servo')
ax2.set_title("Axis 1 - Raw Data")
ax2.set_xlabel("Iteration")
ax2.set_ylabel("Raw Value")
ax2.legend()

# ------------------------------
# AXIS 2
# ------------------------------
axis_2 = data[data['axis'] == 2]
ax3.plot(axis_2['iteration'], axis_2['gyro_measurement'], label='Gyro Measurement')
ax3.plot(axis_2['iteration'], axis_2['rate_command'], label='Rate Command')
ax3.plot(axis_2['iteration'], axis_2['gimbal_axis'], label='Gimbal Axis')
ax3.plot(axis_2['iteration'], axis_2['server_motorized'], label='Servo')
ax3.set_title("Axis 2 - Raw Data")
ax3.set_xlabel("Iteration")
ax3.set_ylabel("Raw Value")
ax3.legend()

fig.tight_layout()
mpl.show()
