import numpy as np
import matplotlib.pyplot as plt

# Parameters
highway_width = 55  # Width of the highway (m)
max_slope = 0.05  # Maximum slope (5%)

# User input
start_height = float(input("Enter height where the cycle path starts (m): "))
end_height = float(input("Enter height where the cycle path ends (m): "))

# Function to generate profile
def generate_profile(passage_type, min_height):
    # Calculate minimum distance needed to meet slope requirement
    length_before = abs(min_height - start_height) / max_slope
    length_after = abs(end_height - min_height) / max_slope

    # Highway points
    highway_start = -highway_width / 2
    highway_end = highway_width / 2

    # Points for each section
    x_left = np.linspace(-length_before - highway_width/2, highway_start, 250)
    x_flat = np.linspace(highway_start, highway_end, 250)
    x_right = np.linspace(highway_end, length_after + highway_width/2, 250)

    # Calculate left curve (sin^2 for smooth start and end)
    t_left = (x_left - x_left[0]) / (x_left[-1] - x_left[0])
    y_left = start_height + (min_height - start_height) * np.sin(np.pi * t_left / 2)**2

    # Flat segment
    y_flat = np.full_like(x_flat, min_height)

    # Calculate right curve (sin^2 for smooth start and end)
    t_right = (x_right - x_right[0]) / (x_right[-1] - x_right[0])
    y_right = min_height + (end_height - min_height) * np.sin(np.pi * t_right / 2)**2

    # Combine full path
    x_total = np.concatenate([x_left, x_flat, x_right])
    y_total = np.concatenate([y_left, y_flat, y_right])

    return x_total, y_total

# Generate profiles for "over" (min clearance height 5 m) and "under" (-3 m)
x_over, y_over = generate_profile("over", min_height=5)
x_under, y_under = generate_profile("under", min_height=-3)

# Plot both path alternatives
plt.figure(figsize=(12, 6))
plt.plot(x_over, y_over, label="Over the highway")
plt.plot(x_under, y_under, label="Under the highway")

# Mark highway
highway_start = -highway_width / 2
highway_end = highway_width / 2
plt.hlines(0, highway_start, highway_end, colors='grey', linewidth=5, label="Highway")

# Mark start and end points
plt.scatter([x_over[0], x_over[-1]], [start_height, end_height], color='red', zorder=5)
plt.scatter([x_under[0], x_under[-1]], [start_height, end_height], color='red', zorder=5, label="Start/End")

plt.xlabel('Distance (m)')
plt.ylabel('Height (m)')
plt.title('Comparison: cycle path over or under highway')
plt.legend()
plt.grid()
plt.axhline(0, color='black', linewidth=0.5)
plt.axvline(0, color='black', linewidth=0.5)
plt.show()
