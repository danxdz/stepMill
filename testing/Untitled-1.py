import matplotlib.pyplot as plt

# define cut path

# create a list of points like a cnc path x and z coordinates and radius for G2 and G3

cnc_path = [
    {'X': 14, 'Z': -77.05, 'radius': 0},
    {'X': 10, 'Z': -71.05, 'radius': 10},
    {'X': 14, 'Z': -35.55, 'radius': 10},
]


# This code computes the intersection of two lines and plots them along with some profile points.



# Main line points
A = [9, -77.05]
B = [7, -71.05]
C = [15, -71.05]
D = [7, -77.05]

Line1 = [A, B]
Line2 = [C, D]

def line_intersection(line1, line2):
    xdiff = (line1[0][0] - line1[1][0], line2[0][0] - line2[1][0])
    ydiff = (line1[0][1] - line1[1][1], line2[0][1] - line2[1][1])

    def det(a, b):
        return a[0] * b[1] - a[1] * b[0]

    div = det(xdiff, ydiff)
    if div == 0:
        raise Exception('lines do not intersect')

    d = (det(*line1), det(*line2))
    x = det(d, xdiff) / div
    y = det(d, ydiff) / div
    return round(x, 3), round(y, 3)

# Compute intersection
ix, iz = line_intersection(Line1, Line2)

# Radius for circles
R2 = 2
R_profile = 10

# Extra profile points
profile_points = [
    {'label': 'P1', 'X': 7, 'Z': -77.05},
    {'label': 'P2 (R10)', 'X': 5, 'Z': -71.05, 'radius': R_profile},
    {'label': 'P3', 'X': 5, 'Z': -40.818}
]

# Plotting
fig, ax = plt.subplots()

# Main lines
ax.plot([A[1], B[1]], [A[0], B[0]], label="Line 1", color="blue")
ax.plot([C[1], D[1]], [C[0], D[0]], label="Line 2", color="green")

# Intersection point
ax.scatter(iz, ix, color='red', label=f'Intersection ({ix}, {iz})', zorder=5)
ax.add_patch(plt.Circle((iz, ix), R2, color='red', fill=False, linestyle='-', alpha=0.8))

# Endpoint circles
for point in [A, B, C, D]:
    circle = plt.Circle((point[1], point[0]), R2, color='gray', fill=False, linestyle='--', alpha=0.6)
    ax.add_patch(circle)

# Plot profile points with annotations
for pt in profile_points:
    z = pt['Z']
    x = pt['X']
    label = pt['label']
    ax.scatter(z, x, color='purple', marker='o', zorder=5)
    ax.annotate(label, (z, x), textcoords="offset points", xytext=(5, 5), ha='left', fontsize=9)
    
    # Optional: draw radius if defined
    if 'radius' in pt:
        circle = plt.Circle((z, x), pt['radius'], color='purple', fill=False, linestyle=':', alpha=0.4)
        ax.add_patch(circle)

# Styling
ax.set_xlabel('Z')
ax.set_ylabel('X')
ax.set_title('Line Intersection with Profile Points')
ax.legend()
ax.grid(True)
ax.set_aspect('equal', adjustable='box')
plt.show()
