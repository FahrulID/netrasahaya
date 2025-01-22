import numpy as np
from collections import deque
from shapely.geometry import Polygon as ShapelyPolygon
from shapely.geometry import Point as ShapelyPoint

import matplotlib.pyplot as plt

kernel = np.array([
    [1, 1],
    [1, 1]
])

# Binary occupancy grid (0 = free, 1 = occupied)
occupancy_grid = np.array([
    [1, 1, 1, 1, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 1],
    [1, 1, 1, 1, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 1],
    [1, 1, 1, 1, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 1],
    [1, 1, 1, 1, 0, 1, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 1],
    [1, 1, 1, 1, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 1],
    [0, 0, 0, 0, 0, 0, 0, 0, 1, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 1],
    [0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 1],
    [0, 0, 0, 0, 0, 0, 0, 1, 0, 1, 0, 0, 0, 0, 0, 0, 0, 0, 0, 1],
    [0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 1],
    [0, 0, 0, 0, 0, 0, 0, 0, 0, 1, 0, 1, 0, 0, 0, 0, 0, 0, 0, 1],
    [0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 1],
    [0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 1, 0, 0, 0, 0, 0, 1],
    [0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 1],
    [0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 1, 0, 1, 0, 0, 0, 1],
    [0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 1],
    [0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 1, 0, 1, 0, 1],
    [0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 1],
    [0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 1, 0, 1],
    [0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 1],
    [0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 1],
])

# create zero array with width of 80 and height of 40
test_grid = np.zeros((40, 80))

def downsample(grid, kernel):
    rows, cols = grid.shape
    k_rows, k_cols = kernel.shape

    # Downsampled grid
    downsampled = np.zeros((rows // k_rows, cols // k_cols))

    for i in range(0, rows, k_rows):
        for j in range(0, cols, k_cols):
            downsampled[i // k_rows, j // k_cols] = np.max(grid[i:i + k_rows, j:j + k_cols])

    return downsampled

def is_path_possible(grid, start_points, end_points, boundary_polygon=None) -> bool:
    """
    Utilizing BFS to find if there is a path from any cell in the first row to any cell in the last row
    :param grid: the grid
    :param start_points: the starting points
    :param end_points: the ending points
    :param boundary_polygon: the boundary polygon
    :return: True if there is a path, False otherwise
    """
    rows, cols = grid.shape

    if not start_points:
        return False  # No free starting points in the last row
    
    if type(boundary_polygon) is not list and boundary_polygon is not None:
        raise ValueError("boundary_polygon must be a list of (x, y) tuples")
    
    boundary_polygon = ShapelyPolygon(boundary_polygon) if boundary_polygon is not None else None

    # Directions for 4-connected grid (up, down, left, right)
    directions = [(-1, 0), (1, 0), (0, -1), (0, 1)]

    # BFS queue and visited set
    queue = deque(start_points)
    visited = set(start_points)

    while queue:
        x, y = queue.popleft()

        # If we reach one of the points then return True
        if (x, y) in end_points:
            return True
        
        print(x, y)

        # Explore neighbors
        for dx, dy in directions:
            nx, ny = x + dx, y + dy
            
            inside_boundary = 0 <= nx < rows and 0 <= ny < cols
            if not inside_boundary:
                continue

            is_visited = (nx, ny) in visited
            if is_visited:
                continue

            inside_polygon = boundary_polygon is None or boundary_polygon.contains(ShapelyPoint(nx, ny)) or boundary_polygon.touches(ShapelyPoint(nx, ny))
            # print(inside_polygon)
            if not inside_polygon:
                continue

            is_occupied = grid[nx, ny] == 1
            if is_occupied:
                continue

            queue.append((nx, ny))
            visited.add((nx, ny))
            downsampled_grid[(x, y)] = 0.5

    return False  # No path found

downsampled_grid = downsample(occupancy_grid, kernel)
print(downsampled_grid)
# boundary = [(0, 0), (5, 0), (5, 10), (0, 10)]
boundary = [(0, 0), (0, 1), (10, 1), (10, 0)]
print(is_path_possible(downsampled_grid, start_points=[(9, 0), (8, 0)], end_points=[(0, 2), (0, 6), (0, 7), (0, 8)], boundary_polygon=boundary))

# downsampled_test_grid = downsample(test_grid, kernel)

# def world_to_grid(x, y, resolution, origin):
#     """
#     Convert a point in the world frame to the cell in the grid map
#     :param x: the x coordinate
#     :param y: the y coordinate
#     :param map_info: the map metadata
#     :return: the cell in the grid map
#     """

#     # rospy.logerr(f'origin: {origin}, resolution: {resolution}')

#     if resolution == 0:
#         return 0, 0

#     grid_x = int((x - origin[0]) / resolution)
#     grid_y = int((y - origin[1]) / resolution)
#     return grid_x, grid_y

# rotate_point = lambda x, y, theta: (x * np.cos(theta) - y * np.sin(theta), x * np.sin(theta) + y * np.cos(theta))

# center_point = (0, 0)
# middle_point = (0, 2)
# middle_point_plus_45 = rotate_point(*middle_point, np.pi/4)
# middle_point_minus_45 = rotate_point(*middle_point, -np.pi/4)
# middle_point_plus_90 = rotate_point(*middle_point, np.pi/2)
# middle_point_minus_90 = rotate_point(*middle_point, -np.pi/2)

# switch_x_y = lambda x, y: (y, x)

# print(center_point, middle_point, middle_point_plus_45)

# origin = (0, 0)
# polygon1 = [
#     world_to_grid(*center_point, 0.05, origin),
#     world_to_grid(*middle_point, 0.05, origin),
#     world_to_grid(*middle_point_plus_45, 0.05, origin),
# ]

# print(polygon1)

# map_data = np.loadtxt('/home/fahrul/netrasahaya/catkin_ws/map.txt')

# map_data[(70,0)] = 1

# # Visualize the grid
# # grid_to_visualize = filter_cells_inside_polygons(test_grid, [polygon1])
grid_to_visualize = downsampled_grid
plt.figure(figsize=grid_to_visualize.shape)  # Adjust figure size
plt.imshow(grid_to_visualize, cmap='gray_r', origin='upper', vmin=0, vmax=1)  # 'gray_r' for reversed colormap, origin='lower' for bottom-left origin
plt.title("Grid Visualization")
plt.xlabel("X-axis")
plt.ylabel("Y-axis")
plt.colorbar()
plt.xticks(np.arange(0.5, grid_to_visualize.shape[1], 1))  # Set x-axis ticks to .5 values
plt.yticks(np.arange(0.5, grid_to_visualize.shape[0], 1))  # Set y-axis ticks to .5 values
plt.grid(True, which='both', color='black', linestyle='-', linewidth=0.5)
plt.show()