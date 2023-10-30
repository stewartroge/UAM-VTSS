import random

import numpy as np


def conflict_detection(ownship, target, D, H, T):
    sx = abs(ownship['position'][0] - target['position'][0])
    sy = abs(ownship['position'][1] - target['position'][1])
    sz = abs(ownship['position'][2] - target['position'][2])
    vx = ownship['velocity'][0] - target['velocity'][0]
    vy = ownship['velocity'][1] - target['velocity'][1]
    vz = ownship['velocity'][2] - target['velocity'][2]

    if vx ** 2 + vy ** 2 == 0:
        if sx ** 2 + sy ** 2 < D ** 2:
            if abs(sz) < H:
                return True
            elif vz * sz < 0 and -H < vz * (T * vz + sz):
                return True
    else:
        a = vx ** 2 + vy ** 2
        b = 2 * (sx * vx + sy * vy)
        c = sx ** 2 + sy ** 2 - D ** 2
        discriminant = b ** 2 - 4 * a * c
        if discriminant >= 0:
            tDin = (-b - np.sqrt(discriminant)) / (2 * a)
            tDout = (-b + np.sqrt(discriminant)) / (2 * a)
            if abs(sz) < H and tDout > 0 and tDin < T:
                return True
            tHin = (H - sz) / vz if vz != 0 else float('inf')
            tHout = (-H - sz) / vz if vz != 0 else float('inf')
            tin = max(tDin, tHin)
            tout = min(tDout, tHout)
            if tin < tout and tout > 0 and tin < T:
                return True
    return False


def VTSS_UAM_algorithm(airspace_volume, UAM_speeds, protection_radius, design_separation, num_corridors, num_layers,
                       max_altitude, T):
    # Calculate corridor width
    corridor_width = (airspace_volume / num_corridors) ** (1 / 3)

    # Calculate lateral separation
    lateral_separation = corridor_width / 2

    # Calculate vertical separation
    vertical_separation = (max_altitude - protection_radius) / (num_layers - 1)
    vertical_separation = max(vertical_separation, protection_radius)

    # Calculate longitudinal separation
    longitudinal_separation = corridor_width

    # Calculate maximum number of vehicles that can fit in the airspace volume
    max_vehicles = int(airspace_volume / (lateral_separation * vertical_separation * longitudinal_separation))

    # Create list to store positions and velocities of UAM vehicles
    vehicle_list = []

    # Generate random initial positions for vehicles within each corridor, layer, and longitudinal segment
    for i in range(num_corridors):
        for j in range(num_layers):
            for k in range(max_vehicles):
                x = i * corridor_width + random.random() * corridor_width
                y = j * vertical_separation + random.random() * vertical_separation
                z = k * longitudinal_separation + random.random() * longitudinal_separation

                initial_position = [x, y, z]
                initial_velocity = [random.uniform(0, UAM_speeds[j]), random.uniform(0, UAM_speeds[j]), 0]

                vehicle_list.append({
                    'position': initial_position,
                    'velocity': initial_velocity,
                    'corridor': i,
                    'layer': j,
                    'longitudinal': z
                })

    # Count the number of conflicts
    conflicts = []
    for i in range(len(vehicle_list)):
        for j in range(i + 1, len(vehicle_list)):
            if conflict_detection(vehicle_list[i], vehicle_list[j], design_separation, vertical_separation, T):
                conflicts.append((i, j))
    num_conflicts = len(conflicts)

    # Calculate conflict rate
    conflict_rate = num_conflicts / (max_vehicles * (max_vehicles - 1) / 2)

    # Calculate time needed to resolve conflicts using 3D distances and velocities
    time_to_resolve = np.max(
        [np.linalg.norm(np.array(vehicle_list[i]['position']) - np.array(vehicle_list[j]['position']))
         / np.linalg.norm(np.array(vehicle_list[i]['velocity']) - np.array(vehicle_list[j]['velocity']))
         for i, j in conflicts]) if conflicts else 0

    # Return the final values
    return vertical_separation, lateral_separation, longitudinal_separation, conflict_rate, time_to_resolve, max_vehicles,


# input parameters
airspace_volume = 80000000
UAM_speeds = [30, 30]  # velocities for each UAM in each layer
protection_radius = 26
design_separation = 30
num_corridors = 6
num_layers = 2
max_altitude = 800
T = 300  # Lookahead time in seconds typical is 5 minutes

# Run the algorithm
vertical_separation, lateral_separation, longitudinal_separation, conflict_rate, time_to_resolve, max_vehicles = VTSS_UAM_algorithm(
    airspace_volume,
    UAM_speeds,
    protection_radius,
    design_separation,
    num_corridors,
    num_layers,
    max_altitude,
    T)

print(f"Conflict Rate: {conflict_rate:.4f}")
print(f"Time to Resolve Conflicts: {time_to_resolve:.2f} seconds")
print(f"Lateral Separation: {lateral_separation:.2f} meters")
print(f"Vertical Separation: {vertical_separation:.2f} meters")
print(f"Longitudinal Separation: {longitudinal_separation:.2f} meters")
print(f"Maximum Number of Vehicles: {max_vehicles}")
