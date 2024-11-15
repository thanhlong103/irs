import pandas as pd
import numpy as np
import matplotlib.pyplot as plt
import heapq
import math

# Parameters for the grid map
grid_size = 30  # Grid dimensions (20x20)
resolution = 0.2  # Size of each grid cell in meters (0.2m x 0.2m)

# Transformation parameters for the slices
rotation_angle_1 = np.radians(3)  # Rotation in radians for the first transformation
translation_vector_1 = np.array([0.55, -0.55])  # Translation for the first slice

rotation_angle_2 = np.radians(90)  # Rotation in radians for the second transformation
translation_vector_2 = np.array([0.6, 0.6])  # Translation for the second slice

# Load all three CSV files containing the LiDAR scan points
df1 = pd.read_csv('data/top.csv')
df2 = pd.read_csv('data/left.csv')
df3 = pd.read_csv('data/right.csv')  # Third map

x1, y1 = df1['x'].values, df1['y'].values
x2, y2 = df2['x'].values, df2['y'].values
x3, y3 = df3['x'].values, df3['y'].values  # For the third map

# Initialize the grid map with zeros (free space)
grid_map = np.zeros((grid_size, grid_size))

# Helper function to convert coordinates to grid indices
def to_grid_indices(x, y, grid_size, resolution):
    grid_x = int((x / resolution) + grid_size // 2)
    grid_y = int((y / resolution) + grid_size // 2)
    return grid_x, grid_y

# Place points from the first slice into the grid map
for x, y in zip(x1, y1):
    grid_x, grid_y = to_grid_indices(x, y, grid_size, resolution)
    if 0 <= grid_x < grid_size and 0 <= grid_y < grid_size:
        grid_map[grid_y, grid_x] = 1  # Mark as occupied

# Define the rotation matrix for the first slice
rotation_matrix_1 = np.array([
    [np.cos(rotation_angle_1), -np.sin(rotation_angle_1)],
    [np.sin(rotation_angle_1), np.cos(rotation_angle_1)]
])

# Apply the transformation to points from the first slice
for x, y in zip(x2, y2):
    # Rotate and translate the point for the second slice
    transformed_point_2 = np.dot(rotation_matrix_1, [x, y]) + translation_vector_1
    # Convert to grid indices
    grid_x, grid_y = to_grid_indices(transformed_point_2[0], transformed_point_2[1], grid_size, resolution)
    if 0 <= grid_x < grid_size and 0 <= grid_y < grid_size:
        grid_map[grid_y, grid_x] = 1  # Mark as occupied

# Define the rotation matrix for the second slice
rotation_matrix_2 = np.array([
    [np.cos(rotation_angle_2), -np.sin(rotation_angle_2)],
    [np.sin(rotation_angle_2), np.cos(rotation_angle_2)]
])

# Apply the transformation to points from the second slice
for x, y in zip(x3, y3):
    # Rotate and translate the point for the third slice
    transformed_point_3 = np.dot(rotation_matrix_2, [x, y]) + translation_vector_2
    # Convert to grid indices
    grid_x, grid_y = to_grid_indices(transformed_point_3[0], transformed_point_3[1], grid_size, resolution)
    if 0 <= grid_x < grid_size and 0 <= grid_y < grid_size:
        grid_map[grid_y, grid_x] = 1  # Mark as occupied

# A* algorithm with diagonal movement
def astar(grid_map, start, goal):
    # Directions for neighbors (up, down, left, right, and diagonals)
    directions = [(0, 1), (1, 0), (0, -1), (-1, 0),  # Cardinal directions
                  (1, 1), (1, -1), (-1, 1), (-1, -1)]  # Diagonal directions
    
    # Initialize the open set as a priority queue with the starting node
    open_set = []
    heapq.heappush(open_set, (0, start))
    
    # Dictionaries to store the g-costs and f-costs
    g_costs = {start: 0}
    f_costs = {start: heuristic(start, goal)}
    
    # Dictionary to store the path
    came_from = {}

    while open_set:
        # Get the node with the lowest f-cost
        _, current = heapq.heappop(open_set)

        # Check if we reached the goal
        if current == goal:
            return reconstruct_path(came_from, current)
        
        # Explore neighbors
        for direction in directions:
            neighbor = (current[0] + direction[0], current[1] + direction[1])
            
            # Skip out-of-bounds or obstacle cells
            if not (0 <= neighbor[0] < grid_size and 0 <= neighbor[1] < grid_size):
                continue
            if grid_map[neighbor[1], neighbor[0]] == 1:  # Occupied cell
                continue
            
            # Calculate g-cost for this neighbor
            tentative_g_cost = g_costs[current] + (1.414 if direction in [(1, 1), (1, -1), (-1, 1), (-1, -1)] else 1)
            
            # Only consider this path if it's the best one found so far
            if neighbor not in g_costs or tentative_g_cost < g_costs[neighbor]:
                came_from[neighbor] = current
                g_costs[neighbor] = tentative_g_cost
                f_costs[neighbor] = tentative_g_cost + heuristic(neighbor, goal)
                heapq.heappush(open_set, (f_costs[neighbor], neighbor))
    
    # Return None if there's no path
    return None

# Heuristic function (Diagonal distance)
def heuristic(point, goal):
    return math.sqrt((point[0] - goal[0])**2 + (point[1] - goal[1])**2)

# Function to reconstruct path
def reconstruct_path(came_from, current):
    path = [current]
    while current in came_from:
        current = came_from[current]
        path.append(current)
    path.reverse()
    return path

# Define start and goal positions in grid coordinates
start = (14, 14)
goal = (20, 14)

# Run A* algorithm
path = astar(grid_map, start, goal)

# Plot the merged grid map and the path
if path:
    # Mark the path on the grid map with a different value (e.g., 0.5 for lighter shade)
    for (x, y) in path:
        grid_map[y, x] = 0.5  # Mark as path
    
    # Visualize the merged grid map with the path
    plt.imshow(grid_map, cmap='Reds', origin='lower')

    # Set grid ticks (center of cells)
    plt.gca().set_xticks(np.arange(0, grid_size, 1))  # Grid lines at cell centers
    plt.gca().set_yticks(np.arange(0, grid_size, 1))

    # Set grid lines at cell edges
    plt.gca().set_xticks(np.arange(-0.5, grid_size, 1), minor=True)
    plt.gca().set_yticks(np.arange(-0.5, grid_size, 1), minor=True)

    # Draw grid lines at cell edges
    plt.gca().grid(which='minor', color='black', linestyle='-', linewidth=0.5)

    plt.title("Merged Grid Map with A* Path")
    plt.xlabel("X (in grid cells)")
    plt.ylabel("Y (in grid cells)")
    plt.show()
else:
    print("No path found from start to goal.")
