import pandas as pd
import numpy as np
import matplotlib.pyplot as plt
import math
import heapq

# Load all four CSV files containing the LiDAR scan points
df1 = pd.read_csv('data/topLeft.csv')
df2 = pd.read_csv('data/topRight.csv')
df3 = pd.read_csv('data/bottomRight.csv')  # Third map
df4 = pd.read_csv('data/bottomLeft.csv')  # Fourth map

x1, y1 = df1['x'].values, df1['y'].values
x2, y2 = df2['x'].values, df2['y'].values
x3, y3 = df3['x'].values, df3['y'].values
x4, y4 = df4['x'].values, df4['y'].values

# Transformation parameters for the slices
rotation_angle_1 = np.radians(1)  # Rotation in radians for the first transformation
translation_vector_1 = np.array([0, 0.8])  # Translation for the first slice

rotation_angle_2 = np.radians(0)  # Rotation in radians for the second transformation
translation_vector_2 = np.array([1, 0.9])  # Translation for the second slice

rotation_angle_3 = np.radians(0)  # Rotation in radians for the third transformation
translation_vector_3 = np.array([1.1, 0])  # Translation for the third slice

# Define rotation matrices
rotation_matrix_1 = np.array([
    [np.cos(rotation_angle_1), -np.sin(rotation_angle_1)],
    [np.sin(rotation_angle_1), np.cos(rotation_angle_1)]
])

rotation_matrix_2 = np.array([
    [np.cos(rotation_angle_2), -np.sin(rotation_angle_2)],
    [np.sin(rotation_angle_2), np.cos(rotation_angle_2)]
])

rotation_matrix_3 = np.array([
    [np.cos(rotation_angle_3), -np.sin(rotation_angle_3)],
    [np.sin(rotation_angle_3), np.cos(rotation_angle_3)]
])

# Apply transformations to align slices to the same frame
transformed_x2, transformed_y2 = np.dot(rotation_matrix_1, np.vstack((x2, y2))) + translation_vector_1[:, np.newaxis]
transformed_x3, transformed_y3 = np.dot(rotation_matrix_2, np.vstack((x3, y3))) + translation_vector_2[:, np.newaxis]
transformed_x4, transformed_y4 = np.dot(rotation_matrix_3, np.vstack((x4, y4))) + translation_vector_3[:, np.newaxis]

# Combine all points into a single dataset
all_x = np.concatenate((x1, transformed_x2, transformed_x3, transformed_x4))
all_y = np.concatenate((y1, transformed_y2, transformed_y3, transformed_y4))

# Determine the bounds of the grid
x_min, x_max = np.min(all_x), np.max(all_x)
y_min, y_max = np.min(all_y), np.max(all_y)

print(x_min, y_min)

# Define the grid resolution
resolution = 0.02  # Size of each grid cell in meters

# Calculate grid dimensions based on bounds and resolution
grid_width = int(np.ceil((x_max - x_min) / resolution))
grid_height = int(np.ceil((y_max - y_min) / resolution))

# Initialize the grid map with zeros (free space)
grid_map = np.zeros((grid_height, grid_width))

# Helper function to convert coordinates to grid indices
def to_grid_indices(x, y, x_min, y_min, resolution):
    grid_x = int((x - x_min) / resolution)
    grid_y = int((y - y_min) / resolution)
    return grid_x, grid_y

# Populate the grid map with points from all slices
for x, y in zip(all_x, all_y):
    grid_x, grid_y = to_grid_indices(x, y, x_min, y_min, resolution)
    if 0 <= grid_x < grid_width and 0 <= grid_y < grid_height:
        grid_map[grid_y, grid_x] = 1  # Mark as occupied

# Visualize the grid map


print(grid_map)
plt.imshow(grid_map, cmap='Reds', origin='lower', extent=[x_min, x_max, y_min, y_max])
plt.title("Merged Grid Map")
plt.xlabel("X (meters)")
plt.ylabel("Y (meters)")
plt.colorbar(label="Occupancy")
plt.show()

# Function to mark cells within a given radius as occupied
def mark_occupied(grid_map, grid_x, grid_y, radius, resolution):
    radius_in_cells = int(np.ceil(radius / resolution))
    for dx in range(-radius_in_cells, radius_in_cells + 1):
        for dy in range(-radius_in_cells, radius_in_cells + 1):
            if dx**2 + dy**2 <= radius_in_cells**2:
                nx, ny = grid_x + dx, grid_y + dy
                if 0 <= nx < grid_map.shape[1] and 0 <= ny < grid_map.shape[0]:
                    grid_map[ny, nx] = 1

# Extend the occupied cells to account for the object radius
object_radius = 0.1  # Radius of the object in meters
for x, y in zip(all_x, all_y):
    grid_x, grid_y = to_grid_indices(x,y, x_min, y_min, resolution)
    if 0 <= grid_x < grid_width and 0 <= grid_y < grid_height:
        mark_occupied(grid_map, grid_x, grid_y, object_radius, resolution)

# Visualize the grid map with occupied cells extended
plt.imshow(grid_map, cmap='Reds', origin='lower', extent=[x_min, x_max, y_min, y_max])
plt.title("Merged Grid Map with Object Radius")
plt.xlabel("X (meters)")
plt.ylabel("Y (meters)")
plt.colorbar(label="Occupancy")
plt.show()

np.save('merged_grid_map.npy', grid_map)

# Define A* algorithm for pathfinding
def astar(grid_map, start, goal):
    directions = [(0, 1), (1, 0), (0, -1), (-1, 0), (1, 1), (1, -1), (-1, 1), (-1, -1)]
    open_set = []
    heapq.heappush(open_set, (0, start))
    g_costs = {start: 0}
    f_costs = {start: heuristic(start, goal)}
    came_from = {}

    while open_set:
        _, current = heapq.heappop(open_set)

        if current == goal:
            return reconstruct_path(came_from, current)

        for direction in directions:
            neighbor = (current[0] + direction[0], current[1] + direction[1])

            if not (0 <= neighbor[0] < grid_map.shape[1] and 0 <= neighbor[1] < grid_map.shape[0]):
                continue
            if grid_map[neighbor[1], neighbor[0]] == 1:
                continue

            tentative_g_cost = g_costs[current] + (1.414 if direction[0] != 0 and direction[1] != 0 else 1)

            if neighbor not in g_costs or tentative_g_cost < g_costs[neighbor]:
                came_from[neighbor] = current
                g_costs[neighbor] = tentative_g_cost
                f_costs[neighbor] = tentative_g_cost + heuristic(neighbor, goal)
                heapq.heappush(open_set, (f_costs[neighbor], neighbor))

    return None

def heuristic(a, b):
    return np.linalg.norm(np.array(a) - np.array(b))

def reconstruct_path(came_from, current):
    path = [current]
    while current in came_from:
        current = came_from[current]
        path.append(current)
    path.reverse()
    return path

# Define start and goal positions in grid coordinates
start = to_grid_indices(-0.4, 0.7, x_min, y_min, resolution)
goal = to_grid_indices(1.5, -0.1, x_min, y_min, resolution)

print(to_grid_indices(0,0,x_min,y_min,resolution))

# Run A* algorithm
path = astar(grid_map, start, goal)

np.save("path.npy", np.array(path))

# Plot the grid map and the path
if path:
    for (x, y) in path:
        grid_map[y, x] = 0.5  # Mark the path with a different value

    plt.imshow(grid_map, cmap='Reds', origin='lower', extent=[x_min, x_max, y_min, y_max])
    plt.title("Merged Grid Map with A* Path")
    plt.xlabel("X (meters)")
    plt.ylabel("Y (meters)")
    plt.colorbar(label="Occupancy")
    plt.show()
else:
    print("No path found from start to goal.")
