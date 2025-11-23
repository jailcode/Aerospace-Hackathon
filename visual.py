from matplotlib import pyplot as mpl;
import pandas as pd;

data = pd.read_csv("results.csv");
print(data[['iteration', 'gyro_measurement', 'rate_command']].head())
mpl.plot(data['iteration'], data['gyro_measurement'], label='Comand')
mpl.plot(data['iteration'], -data['rate_command'], label='Measurement')
mpl.plot(data['iteration'], data['gimbal_axis'], label='gimbal')
mpl.xlabel('Iteration')
mpl.ylabel('Value')
mpl.title('Gyro vs Expected Rate Command')
mpl.legend()
mpl.show()
