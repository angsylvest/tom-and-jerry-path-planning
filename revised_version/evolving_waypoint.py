import numpy as np
import matplotlib.pyplot as plt

def generate_circle_points(center, radius, num_points=100):
    theta = np.linspace(0, 2*np.pi, num_points)
    x = center[0] + radius * np.cos(theta)
    y = center[1] + radius * np.sin(theta)
    return x, y

def distance(point1, point2):
    return np.sqrt((point1[0] - point2[0])**2 + (point1[1] - point2[1])**2)

def find_tangent_point(center, radius, point):
    angle = np.arctan2(point[1] - center[1], point[0] - center[0])
    x = center[0] + radius * np.cos(angle)
    y = center[1] + radius * np.sin(angle)
    return x, y

def order_coordinates_quadrants(coordinates):
    # Order the coordinates based on the highest x value within each quadrant
    # Quadrant I: (+, +)
    quadrant_I = coordinates[(coordinates[:, 0] >= 0) & (coordinates[:, 1] >= 0)]
    quadrant_I = quadrant_I[np.lexsort((quadrant_I[:, 1], quadrant_I[:, 0]))[::-1]]

    # Quadrant II: (-, +)
    quadrant_II = coordinates[(coordinates[:, 0] < 0) & (coordinates[:, 1] >= 0)]
    quadrant_II = quadrant_II[np.lexsort((quadrant_II[:, 1], quadrant_II[:, 0]))[::-1]]

    # Quadrant III: (-, -)
    quadrant_III = coordinates[(coordinates[:, 0] < 0) & (coordinates[:, 1] < 0)]
    quadrant_III = quadrant_III[np.lexsort((quadrant_III[:, 1], quadrant_III[:, 0]))]

    # Quadrant IV: (+, -)
    quadrant_IV = coordinates[(coordinates[:, 0] >= 0) & (coordinates[:, 1] < 0)]
    quadrant_IV = quadrant_IV[np.lexsort((quadrant_IV[:, 1], quadrant_IV[:, 0]))]

    return np.concatenate([quadrant_I, quadrant_II, quadrant_III, quadrant_IV])

def get_circle_paths_and_coordinates(center, radius, start_pos, end_pos, num_coordinates=20):
    if distance(center, start_pos) <= radius or distance(center, end_pos) <= radius:
        raise ValueError("Start and end positions must be outside the circle.")
    
    # Find tangent points on the circle
    tangent_start = find_tangent_point(center, radius, start_pos)
    tangent_end = find_tangent_point(center, radius, end_pos)
    
    # Generate coordinates around the circle
    theta_coordinates = np.linspace(0, 2*np.pi, num_coordinates, endpoint=False)
    coordinates = np.array([
        center[0] + radius * np.cos(theta_coordinates),
        center[1] + radius * np.sin(theta_coordinates)
    ]).T
    
    # Add start and end paths to the coordinates list
    coordinates = np.concatenate([coordinates, [tangent_start, tangent_end]])
    
    # Find the index where the end of the start path is located
    index_end_of_start_path = np.where((coordinates[:, 0] == tangent_start[0]) & (coordinates[:, 1] == tangent_start[1]))[0][0]
    
    # Start the coordinates list from the end of the start path
    coordinates = np.roll(coordinates, -index_end_of_start_path, axis=0)
    
    # Order the coordinates based on the highest x value within each quadrant
    ordered_coordinates = order_coordinates_quadrants(coordinates)
    
    return tangent_start, tangent_end, ordered_coordinates

def plot_circle_with_paths_and_coordinates(center, radius, start_pos, end_pos, tangent_start, tangent_end, coordinates):
    x_circle, y_circle = generate_circle_points(center, radius)
    
    plt.figure(figsize=(8, 8))
    
    # Plot circle
    plt.plot(x_circle, y_circle, label='Circle')
    
    # Plot center
    plt.scatter(center[0], center[1], color='red', label='Center')
    
    # Plot start and end positions
    plt.scatter(start_pos[0], start_pos[1], color='green', label='Start Position')
    plt.scatter(end_pos[0], end_pos[1], color='blue', label='End Position')
    
    # Plot paths from start and end positions to the tangent points
    plt.plot([start_pos[0], tangent_start[0]], [start_pos[1], tangent_start[1]], linestyle='--', color='green', label='Start Path')
    plt.plot([end_pos[0], tangent_end[0]], [end_pos[1], tangent_end[1]], linestyle='--', color='blue', label='End Path')
    
    # Mark the spots where the start and end paths end
    plt.scatter(tangent_start[0], tangent_start[1], color='green', marker='x', label='End of Start Path')
    plt.scatter(tangent_end[0], tangent_end[1], color='blue', marker='x', label='End of End Path')
    
    # Plot coordinates around the circle
    plt.scatter(coordinates[:, 0], coordinates[:, 1], color='orange', label='Marked Coordinates', marker='o')
    
    # Mark the start and end points of the coordinate list
    plt.scatter(coordinates[0, 0], coordinates[0, 1], color='purple', marker='s', label='Start of Coordinates')
    plt.scatter(coordinates[-1, 0], coordinates[-1, 1], color='purple', marker='D', label='End of Coordinates')
    
    # Plot path following the marked coordinates
    plt.plot(coordinates[:, 0], coordinates[:, 1], linestyle='-', color='purple', label='Path following Coordinates')
    
    # Annotate the start and end paths in the coordinates list
    plt.annotate('Start Path', xy=start_pos, xytext=(start_pos[0] + 1, start_pos[1] + 1),
                 arrowprops=dict(facecolor='black', arrowstyle='->'), fontsize=8, color='green')
    plt.annotate('End Path', xy=end_pos, xytext=(end_pos[0] - 1, end_pos[1] - 1),
                 arrowprops=dict(facecolor='black', arrowstyle='->'), fontsize=8, color='blue')
    
    plt.title(f'Circle with Radius {radius} at Center {center}')
    plt.xlabel('X-axis')
    plt.ylabel('Y-axis')
    plt.legend()
    plt.grid(True)
    plt.show()

# Example usage with corrected positions:
center = (5, 1)
radius = 1
start_pos = (7, 2)  # Adjusted to be outside the circle
end_pos = (-6, -3)  # Adjusted to be outside the circle

tangent_start, tangent_end, marked_coordinates = get_circle_paths_and_coordinates(center, radius, start_pos, end_pos) # FID radius, center is where obstacle is 

# Print the values
print("Tangent Start Point:", tangent_start)
print("Tangent End Point:", tangent_end)
print("Marked Coordinates around the Circle (excluding start and end positions):")
print(marked_coordinates)

# Plot the graph
plot_circle_with_paths_and_coordinates(center, radius, start_pos, end_pos, tangent_start, tangent_end, marked_coordinates)
