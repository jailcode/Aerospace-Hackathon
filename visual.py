import pandas as pd
from matplotlib import pyplot as mpl

# Read the data
data = pd.read_csv("results.csv")

# Clean column names to remove any leading/trailing spaces
data.columns = data.columns.str.strip()

# Create subplots with 3 rows and 1 column (for each axis 0, 1, 2)
fig, (ax1, ax2, ax3) = mpl.subplots(3, 1, figsize=(10, 15))

# Plot for Axis 0
axis_0_data = data[data['axis'] == 0]
ax1.plot(axis_0_data['iteration'], axis_0_data['gyro_measurement'], label='Gyro Measurement', color='blue')
ax1.plot(axis_0_data['iteration'], axis_0_data['inner_setpoint'], label='Setpoint', color='red')
ax1.plot(axis_0_data['iteration'], axis_0_data['gimbal_axis'], label='Gimbal Axis', color='green')
ax1.plot(axis_0_data['iteration'], axis_0_data['desired_angular_acceleration'], label='Desired Angular Acceleration', color='purple')
ax1.set_title('Axis 0 - Gyro, Rate, and Gimbal')
ax1.set_xlabel('Iteration')
ax1.set_ylabel('Value')
ax1.legend()

# Plot for Axis 1 (Inverting `gyro_measurement` and `desired_angular_acceleration`)
axis_1_data = data[data['axis'] == 1]

ax2.plot(axis_1_data['iteration'], axis_1_data['desired_angular_acceleration'], label='Gyro Measurement', color='blue')
ax2.plot(axis_1_data['iteration'], axis_1_data['inner_setpoint'], label='Setpoint', color='red')
ax2.plot(axis_1_data['iteration'], axis_1_data['gimbal_axis'], label='Gimbal Axis', color='green')
ax2.plot(axis_1_data['iteration'], axis_1_data['gyro_measurement'], label='Desired Angular Acceleration', color='purple')
ax2.set_title('Axis 1 - Gyro, Rate, and Gimbal (Inverted Gyro and Angular Acceleration)')
ax2.set_xlabel('Iteration')
ax2.set_ylabel('Value')
ax2.legend()

# Plot for Axis 2
axis_2_data = data[data['axis'] == 2]
ax3.plot(axis_2_data['iteration'], axis_2_data['gyro_measurement'], label='Gyro Measurement', color='blue')
ax3.plot(axis_2_data['iteration'], axis_2_data['inner_setpoint'], label='Setpoint', color='red')
ax3.plot(axis_2_data['iteration'], axis_2_data['gimbal_axis'], label='Gimbal Axis', color='green')
ax3.plot(axis_2_data['iteration'], axis_2_data['desired_angular_acceleration'], label='Desired Angular Acceleration', color='purple')
ax3.set_title('Axis 2 - Gyro, Rate, and Gimbal')
ax3.set_xlabel('Iteration')
ax3.set_ylabel('Value')
ax3.legend()

# Adjust the layout to prevent overlap
fig.tight_layout()

# Show the plots
mpl.show()
