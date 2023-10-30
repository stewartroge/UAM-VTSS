import numpy as np
import matplotlib.pyplot as plt
from mpl_toolkits.mplot3d import Axes3D

def plot_3d_trajectories_with_obstacles(aircraft1, aircraft2, obstacles):
    # Create a new figure and set up 3D axes
    fig = plt.figure()
    ax = fig.add_subplot(111, projection='3d')

    # Set limits for the airspace
    airspace_limits = 2000  # Size of the airspace
    ax.set_xlim([0, airspace_limits])
    ax.set_ylim([0, airspace_limits])
    ax.set_zlim([0, airspace_limits])

    # Set labels for the axes
    ax.set_xlabel('X')
    ax.set_ylabel('Y')
    ax.set_zlabel('Z')

    # Plot the trajectories of the aircraft
    ax.plot(aircraft1['position'][:, 0], aircraft1['position'][:, 1], aircraft1['position'][:, 2], color='red', label='Ownship')
    ax.plot(aircraft2['position'][:, 0], aircraft2['position'][:, 1], aircraft2['position'][:, 2], color='blue', label='Rogue UAM')

    # Plot the boxes representing the aircraft
    aircraft_box_size = 100  # Size of the aircraft box
    for i in range(len(aircraft1['position'])):
        ax.bar3d(aircraft1['position'][i, 0], aircraft1['position'][i, 1], aircraft1['position'][i, 2],
                 aircraft_box_size, aircraft_box_size, aircraft_box_size, color='red', alpha=0.2)
        ax.bar3d(aircraft2['position'][i, 0], aircraft2['position'][i, 1], aircraft2['position'][i, 2],
                 aircraft_box_size, aircraft_box_size, aircraft_box_size, color='blue', alpha=0.2)

    # Plot the obstacles as boxes
    obstacle_box_size = 100  # Size of the obstacle box
    for obstacle in obstacles:
        x, y, height = obstacle
        ax.bar3d(x - obstacle_box_size/2, y - obstacle_box_size/2, 0, obstacle_box_size, obstacle_box_size, height + 100,
                 color='gray', alpha=0.5)

    # Add a legend
    ax.legend()

    # Show the plot
    plt.show()


# Define aircraft trajectories (sample data)
time_steps = 5
aircraft1_position = np.random.rand(time_steps, 3) * 2000  # Random positions within the airspace
aircraft2_position = np.random.rand(time_steps, 3) * 2000  # Random positions within the airspace

# Create aircraft dictionaries
aircraft1 = {'position': aircraft1_position}
aircraft2 = {'position': aircraft2_position}

# Generate random obstacle buildings
num_obstacles = 10
obstacles = []
for _ in range(num_obstacles):
    x = np.random.uniform(0, 2000)
    y = np.random.uniform(0, 2000)
    height = np.random.uniform(0, 500)
    obstacles.append((x, y, height))

# Get the maximum height of the buildings
max_building_height = max(obstacle[2] for obstacle in obstacles)

# Modify the maximum height of the aircraft
max_aircraft_height = max_building_height + 100

# Adjust the z-coordinate of the aircraft positions
aircraft1['position'][:, 2] = max_aircraft_height
aircraft2['position'][:, 2] = max_aircraft_height

# Plot the trajectories with obstacles
plot_3d_trajectories_with_obstacles(aircraft1, aircraft2, obstacles)
