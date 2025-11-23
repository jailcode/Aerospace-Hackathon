from matplotlib import pyplot as mpl;
import pandas as pd;

data = pd.read_csv("results.csv");

fig, (ax1, ax2, ax3) = mpl.subplots(3, 1, figsize=(10, 15))

axis_0_data = data[data['axis'] == 0]
ax1.plot(axis_0_data['iteration'], axis_0_data['gyro_measurement'], label='Gyro Measurement', color='blue')
ax1.plot(axis_0_data['iteration'], axis_0_data['rate_command'], label='Rate Command', color='red')
ax1.plot(axis_0_data['iteration'], axis_0_data['gimbal_axis'], label='Gimbal Axis', color='green')
ax1.plot(axis_0_data['iteration'], axis_0_data['server_motorized'], label='Servo', color='yellow')
ax1.set_title('Axis 0 - Measurements, Commands & Gimbal')
ax1.set_xlabel('Iteration')
ax1.set_ylabel('Value')
ax1.legend()

axis_1_data = data[data['axis'] == 1]
ax2.plot(axis_1_data['iteration'], axis_1_data['gyro_measurement'], label='Gyro Measurement', color='blue')
ax2.plot(axis_1_data['iteration'], axis_1_data['rate_command'], label='Rate Command', color='red')
ax2.plot(axis_1_data['iteration'], axis_1_data['gimbal_axis'], label='Gimbal Axis', color='green')
ax2.plot(axis_1_data['iteration'], axis_1_data['server_motorized'], label='Servo', color='yellow')
ax2.set_title('Axis 1 - Measurements, Commands & Gimbal')
ax2.set_xlabel('Iteration')
ax2.set_ylabel('Value')
ax2.legend()

axis_2_data = data[data['axis'] == 2]
ax3.plot(axis_2_data['iteration'], axis_2_data['gyro_measurement'], label='Gyro Measurement', color='blue')
ax3.plot(axis_2_data['iteration'], axis_2_data['rate_command'], label='Rate Command', color='red')
ax3.plot(axis_2_data['iteration'], axis_2_data['gimbal_axis'], label='Gimbal Axis', color='green')
ax3.plot(axis_2_data['iteration'], axis_2_data['server_motorized'], label='Servo', color='yellow')
ax3.set_title('Axis 2 - Measurements, Commands & Gimbal')
ax3.set_xlabel('Iteration')
ax3.set_ylabel('Value')
ax3.legend()

fig.tight_layout()

mpl.show()
