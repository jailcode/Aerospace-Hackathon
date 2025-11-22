#!/usr/bin/env python3
"""
Simple visualization of the rocket control system output
Shows actuator commands over time
"""

import matplotlib.pyplot as plt
import numpy as np

# Read output data
data = np.loadtxt('output.txt', delimiter=',')

gimbal_x = data[:, 0]
gimbal_y = data[:, 1]
fin_angle = data[:, 2]

iterations = np.arange(len(gimbal_x))

# Create figure with subplots
fig, axes = plt.subplots(3, 1, figsize=(10, 8))
fig.suptitle('Rocket TVC Control System - Actuator Commands', fontsize=14, fontweight='bold')

# Plot Gimbal X (Pitch Control)
axes[0].plot(iterations, gimbal_x, 'b-', linewidth=2, label='Gimbal X')
axes[0].axhline(y=0, color='r', linestyle='--', alpha=0.3)
axes[0].set_ylabel('Angle (degrees)', fontsize=10)
axes[0].set_title('Gimbal X - Pitch Control')
axes[0].grid(True, alpha=0.3)
axes[0].legend()

# Plot Gimbal Y (Yaw Control)
axes[1].plot(iterations, gimbal_y, 'g-', linewidth=2, label='Gimbal Y')
axes[1].axhline(y=0, color='r', linestyle='--', alpha=0.3)
axes[1].set_ylabel('Angle (degrees)', fontsize=10)
axes[1].set_title('Gimbal Y - Yaw Control')
axes[1].grid(True, alpha=0.3)
axes[1].legend()

# Plot Fin Angle (Roll Control)
axes[2].plot(iterations, fin_angle, 'm-', linewidth=2, label='Fin Angle')
axes[2].axhline(y=0, color='r', linestyle='--', alpha=0.3)
axes[2].set_xlabel('Iteration', fontsize=10)
axes[2].set_ylabel('Angle (degrees)', fontsize=10)
axes[2].set_title('Fin Angle - Roll Control')
axes[2].grid(True, alpha=0.3)
axes[2].legend()

plt.tight_layout()
plt.savefig('control_output.png', dpi=150)
print("Visualization saved to control_output.png")

# Statistics
print("\n=== Control Output Statistics ===")
print(f"Gimbal X: mean={np.mean(gimbal_x):.4f}°, std={np.std(gimbal_x):.4f}°, range=[{np.min(gimbal_x):.4f}, {np.max(gimbal_x):.4f}]")
print(f"Gimbal Y: mean={np.mean(gimbal_y):.4f}°, std={np.std(gimbal_y):.4f}°, range=[{np.min(gimbal_y):.4f}, {np.max(gimbal_y):.4f}]")
print(f"Fin Angle: mean={np.mean(fin_angle):.4f}°, std={np.std(fin_angle):.4f}°, range=[{np.min(fin_angle):.4f}, {np.max(fin_angle):.4f}]")
