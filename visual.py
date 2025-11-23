from matplotlib import pyplot as mpl
import pandas as pd

data = pd.read_csv("results.csv")

fig, (ax1, ax2, ax3) = mpl.subplots(3, 1, figsize=(10, 15))

def plot_axis(ax, axis_data, title):
    ax.plot(axis_data['iteration'], axis_data['gyro_measurement'], label='Gyro Measurement')
    ax.plot(axis_data['iteration'], axis_data['rate_command'], label='Rate Command')
    ax.plot(axis_data['iteration'], axis_data['gimbal_axis'], label='Gimbal Axis')
    ax.plot(axis_data['iteration'], axis_data['desired_angular_acceleration'], label='Desired Angular Acceleration')
    ax.plot(axis_data['iteration'], axis_data['actuator_command'], label='Actuator Command')
    ax.set_title(title)
    ax.set_xlabel("Iteration")
    ax.set_ylabel("Value")
    ax.legend()

# ------------------------------
# AXIS 0
# ------------------------------
axis_0 = data[data['axis'] == 0]
plot_axis(ax1, axis_0, "Axis 0 - Raw Data")

# ------------------------------
# AXIS 1
# ------------------------------
axis_1 = data[data['axis'] == 1]
plot_axis(ax2, axis_1, "Axis 1 - Raw Data")

# ------------------------------
# AXIS 2
# ------------------------------
axis_2 = data[data['axis'] == 2]
plot_axis(ax3, axis_2, "Axis 2 - Raw Data")

fig.tight_layout()
mpl.show()
