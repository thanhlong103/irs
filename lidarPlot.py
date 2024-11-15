import pandas as pd
import matplotlib.pyplot as plt

# Read data from CSV
file_path = 'lidar.csv'  # Replace with the path to your CSV file
file_path2 = 'coordinates.csv'
data = pd.read_csv(file_path)
data2 = pd.read_csv(file_path2)

# Plotting the data
plt.figure(figsize=(8, 6))
plt.scatter(data['x'], data['y'], color='blue', label='Data points')
plt.scatter(data2['X'], data2['Y'], color='red', label='Data points')

# Set axis limits
plt.xlim(-2, 2)   # Set x-axis limits
plt.ylim(-3, 0)  # Set y-axis limits

# Labels and title
plt.xlabel('X values')
plt.ylabel('Y values')
plt.title('Plot of X vs Y')

# Add legend and grid
plt.legend()
plt.grid(True)

# Show the plot
plt.show()
