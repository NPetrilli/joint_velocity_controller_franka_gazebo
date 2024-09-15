import pandas as pd
import matplotlib.pyplot as plt
import numpy as np
import rosbag

# Read CSV files using pandas to handle headers
M = pd.read_csv('/home/vandalsnike/Results/trajectory_done.csv')
M2 = pd.read_csv('/home/vandalsnike/Results/trajectory.csv')

# Convert DataFrames to NumPy arrays
M = M.to_numpy()
M2 = M2.to_numpy()

# Plot the trajectories
plt.figure(1)
plt.plot(M[:, 1], M[:, 3], 'r', linewidth=2)
plt.axis('equal')
plt.plot(M2[:, 1], M2[:, 3], 'b-.', linewidth=2)
plt.title("Trajectory Data: Trefoil Knot in XZ Plane")
plt.scatter(M2[0, 1], M2[0, 3], color='m', marker='*')
plt.scatter(M2[-1, 1], M2[-1, 3], color='g', marker='*')
plt.legend(['Required', 'Executed', 'Starting point', 'Ending point'])
plt.xlabel("X")
plt.ylabel("Z")
plt.grid(True)

# Read rosbag
bag = rosbag.Bag("/home/vandalsnike/Results/joint_states_record_2024-09-05-22-13-46.bag")
joints_positions = []
joints_velocities = []

for topic, msg, t in bag.read_messages(topics=['/joint_states']):
    time = t.to_sec()
    positions = list(msg.position[:7])  # Extracting first 7 positions
    velocities = list(msg.velocity[:7])  # Extracting first 7 velocities
    joints_positions.append([time] + positions)
    joints_velocities.append([time] + velocities)

bag.close()

joints_positions = np.array(joints_positions)
joints_velocities = np.array(joints_velocities)

# Plot positions and velocities
plt.figure(2)
plt.subplot(1, 2, 1)
for i in range(1, 8):
    plt.scatter(joints_positions[:, 0], joints_positions[:, i], label=f'joint{i}', s=10)
plt.title("Joints' positions")
plt.legend()
plt.xlabel('Time')
plt.grid(True)

plt.subplot(1, 2, 2)
for i in range(1, 8):
    plt.plot(joints_velocities[:, 0], joints_velocities[:, i], label=f'joint{i}')
plt.title("Joints' velocity")
plt.legend()
plt.xlabel('Time')
plt.grid(True)

plt.tight_layout()
plt.show()
