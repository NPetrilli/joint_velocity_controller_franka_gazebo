import numpy as np
import matplotlib.pyplot as plt
import math

# Parameters
scale = 0.05
omega = 0.2
center = np.array([0.5, 0.0, 0.5])

# Time variable
current_time = np.linspace(0, 50, 1000)  # Simulate from t=0 to t=50 with 1000 points
theta = omega * current_time

# Trajectory calculations
x = center[0] + scale * np.sin(theta) + 2 * scale * np.sin(2 * theta)
y = center[1]
z = center[2] + scale * np.cos(theta) - 2 * scale * np.cos(2 * theta)

x = center[0] + scale * np.sin(omega * current_time + np.pi / 2) + 2 * scale * np.sin(2 * omega * current_time + np.pi / 2)
y = center[1]
z = center[2] + scale * np.cos(omega * current_time + np.pi / 2) - 2 * scale * np.cos(2 * omega * current_time + np.pi / 2)


# Plotting the trajectory in the 2D XZ plane with equal axes
plt.figure(figsize=(10, 6))
plt.plot(x, z, label='Trajectory in XZ plane')

plt.xlabel('X')
plt.ylabel('Z')
plt.title('2D Trajectory Plot in XZ Plane')
plt.legend()
plt.grid(True)
plt.axis('equal')  # Set equal scaling by changing axis limits

plt.show()

