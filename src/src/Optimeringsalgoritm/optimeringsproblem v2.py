import numpy as np
import matplotlib.pyplot as plt
from NUMBA_BikeEnergyModelOPT import BikeEnergyModel

# Parameters
highway_width = 55  # Width of the highway (m)
max_slope = 0.05  # Maximum slope (5%)

# User input
start_height = float(input("Enter height where the cycle path starts (m): "))
end_height = float(input("Enter height where the cycle path ends (m): "))

# Function to generate profile
def generate_profile(passage_type, min_height):
    length_before = abs(min_height - start_height) / max_slope
    length_after = abs(end_height - min_height) / max_slope

    highway_start = -highway_width / 2
    highway_end = highway_width / 2

    x_left = np.linspace(-length_before - highway_width/2, highway_start, 250)
    x_flat = np.linspace(highway_start, highway_end, 250)
    x_right = np.linspace(highway_end, length_after + highway_width/2, 250)

    t_left = (x_left - x_left[0]) / (x_left[-1] - x_left[0])
    z_left = start_height + (min_height - start_height) * np.sin(np.pi * t_left / 2)**2

    z_flat = np.full_like(x_flat, min_height)

    t_right = (x_right - x_right[0]) / (x_right[-1] - x_right[0])
    z_right = min_height + (end_height - min_height) * np.sin(np.pi * t_right / 2)**2

    x_total = np.concatenate([x_left, x_flat, x_right])
    z_total = np.concatenate([z_left, z_flat, z_right])

    return x_total, z_total

# Generate profiles
x_over, z_over = generate_profile("over", min_height=5)
x_under, z_under = generate_profile("under", min_height=-3)

coordinates_over = [(float(x), 0.0, float(z)) for x, z in zip(x_over, z_over)]
coordinates_under = [(float(x), 0.0, float(z)) for x, z in zip(x_under, z_under)]

# Run energy model
power = 150.0
mass = 70.0
c_r = 0.007
cwxA = 0.45

E_over, T_over, D_over, Vavg_over = BikeEnergyModel(power, mass, c_r, cwxA, map_file_path=coordinates_over)
E_under, T_under, D_under, Vavg_under = BikeEnergyModel(power, mass, c_r, cwxA, map_file_path=coordinates_under)

# Print results
print("--- OVER THE HIGHWAY ---")
print(f"Energy: {E_over:.1f} J | Time: {T_over:.1f} s | Distance: {D_over:.1f} m | Avg Speed: {Vavg_over*3.6:.1f} km/h")
print("--- UNDER THE HIGHWAY ---")
print(f"Energy: {E_under:.1f} J | Time: {T_under:.1f} s | Distance: {D_under:.1f} m | Avg Speed: {Vavg_under*3.6:.1f} km/h")

# Plot profiles
plt.figure(figsize=(12, 6))
plt.plot(x_over, z_over, label="Over the highway")
plt.plot(x_under, z_under, label="Under the highway")

highway_start = -highway_width / 2
highway_end = highway_width / 2
plt.hlines(0, highway_start, highway_end, colors='grey', linewidth=5, label="Highway")

plt.scatter([x_over[0], x_over[-1]], [start_height, end_height], color='red', zorder=5)
plt.scatter([x_under[0], x_under[-1]], [start_height, end_height], color='red', zorder=5, label="Start/End")

plt.xlabel('Distance (m)')
plt.ylabel('Height (z, m)')
plt.title('Comparison: cycle path over or under highway')
plt.legend()
plt.grid()
plt.axhline(0, color='black', linewidth=0.5)
plt.axvline(0, color='black', linewidth=0.5)
plt.show()
