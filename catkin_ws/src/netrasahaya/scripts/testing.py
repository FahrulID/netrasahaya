from shapely.geometry import Polygon, Point

# Define a polygon with more than 4 points (example: hexagon)
polygon = Polygon([(0.19305144250392914, 0.8492285013198853), (0.13128908478622267, 1.3453992493181426), (0.5002245610756388, 1.243746783820114)])

# Define grid spacing
grid_spacing = 1

# Get the bounding box of the polygon
min_x, min_y, max_x, max_y = polygon.bounds

# Generate grid points
grid_points = []
x = min_x
while x <= max_x:
    y = min_y
    while y <= max_y:
        point = Point(x, y)
        # Include points inside or on the edge of the polygon
        if polygon.contains(point) or polygon.touches(point):
            grid_points.append((x, y))
        y += grid_spacing
    x += grid_spacing

# Output the valid points
print("Valid points inside or on the edges of the polygon:")
for point in grid_points:
    print(point[0])
