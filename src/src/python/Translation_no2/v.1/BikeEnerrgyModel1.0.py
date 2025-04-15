def bike_energy_model(CyclistPowerIn, CyclistMassIn, CrIn, cwxA, distances, elevations):
    g = 9.81  # gravity
    rho = 1.225  # air density at 15 degrees Celsius
    bike_mass = 18.3  # mass of bike+luggage

    total_mass = CyclistMassIn + bike_mass
    velocity = 5.0  # initial velocity in m/s (set as per MATLAB script)
    
    Energy = 0
    Time = 0
    vx_total = []

    # Iterate over route segments
    for i in range(1, len(distances)):
        dx = distances[i] - distances[i-1]
        dz = elevations[i] - elevations[i-1]
        slope = dz / dx if dx != 0 else 0

        alpha = np.arctan(slope)

        # Resistances
        F_roll = total_mass * g * CrIn * np.cos(alpha)
        F_air = 0.5 * rho * cwxA * velocity ** 2
        F_slope = total_mass * g * np.sin(alpha)

        # total resistance
        F_total = F_roll + F_air + F_slope

        # Power required
        P_required = F_total * velocity

        # Determine acceleration/deceleration based on power available
        P_available = CyclistPowerIn
        F_available = P_available / velocity if velocity > 0 else 0

        # acceleration (simplified, ignoring dynamics detailed in MATLAB script)
        acceleration = (F_available - F_total) / total_mass

        # Update velocity
        velocity_new = np.sqrt(max(velocity ** 2 + 2 * acceleration * dx, 0.01))
        
        # Update time and energy
        avg_velocity = (velocity + velocity_new) / 2
        dt = dx / avg_velocity

        Time += dt
        Energy += P_required * dt

        velocity = velocity_new
        vx_total.append(velocity)

    Distance = distances[-1]
    AvgSpeed = Distance / Time

    return Energy, Time, Distance, AvgSpeed
