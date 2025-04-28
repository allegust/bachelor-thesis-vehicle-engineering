def Free_Rolling_Slope(m, f_r, cwxA, rho):
    """
    FreeRollingSlope calculates a free rolling speed
    from which angle will the bike accelerate
    FreeRollingSlope calculates a characteristic or a Function alpha(vx) valid for an individual cyclist
      alpha is the slope (negative alpha -> downhill)
      and depending on vx the value for alpha varies
      The output is a characteristic that can be used in the main programme
    """

    import math
    import numpy as np

    # Constants
    g = 9.81  # [m/s^2]

    # Theory
    # Power to overcome rolling resistance:
    # P_rr = m * g * f_r * cos(alpha) * vx
    # Air resistance power:
    # P_ad = 0.5 * cwxA * rho * vx^3
    # No acceleration power -> free rolling or accelerating by means of downhill-slope force!!
    # Downhill (dh) -> alpha has negative sign!
    # P_dh = m * g * sin(alpha) * vx  % This expression will be negative
    # For which alpha and vx is P_rr + P_ad = P_dh ?
    # F = m * g * f_r * cos(alpha) * vx + 0.5 * cwxA * rho * vx^3 + m * g * sin(alpha) * vx

    # Creating a speed vector to run through
    #vx_Vector = np.linspace(0.1, 20.0, int((20.0-0.1)/0.1)+1)  # guarantees 10.5
    vx_Vector = np.arange(0.1, 20.0 + 1e-9, 0.1)

    alpha_Vector = []

    # Define the sub-function Alpha_vx (translated exactly)
    def Alpha_vx(m, f_r, vx, cwxA, rho):
        """
        Alpha_vx optimises alpha to a certain vx
        """

        g = 9.81  # [m/s^2]

        # alpha = [-0.001:-0.1:-0.901];
        alpha_list = []
        for i in range(10):  # i=0..9
            alpha_list.append(-0.001 - 0.1 * i)

        F = []
        for cc in range(10):
            aa = alpha_list[cc]
            F_val = (m * g * f_r * math.cos(aa) * vx
                     + 0.5 * cwxA * rho * (vx ** 3)
                     + m * g * math.sin(aa) * vx)
            F.append(F_val)

        # [F_min,F_min_index] = min(abs(F));
        # We find the index of the minimal absolute value in F
        absF = [abs(x) for x in F]
        F_min = min(absF)
        F_min_index = absF.index(F_min)  # 0-based

        # if F_min_index==1 || abs(F(F_min_index+1)) < abs(F(F_min_index-1))
        if F_min_index == 0 or (F_min_index < 9 and abs(F[F_min_index + 1]) < abs(F[F_min_index - 1])):
            alpha_max = alpha_list[F_min_index + 1]
            alpha_min = alpha_list[F_min_index]
        else:
            alpha_max = alpha_list[F_min_index]
            alpha_min = alpha_list[F_min_index - 1]

        # alpha = [alpha_min:((alpha_max-alpha_min)/10):alpha_max];
        alpha2 = []
        step_val = (alpha_max - alpha_min) / 10.0
        val = alpha_min
        for _ in range(11):
            alpha2.append(val)
            val += step_val

        F2 = []
        for cc in range(11):
            aa = alpha2[cc]
            F_val = (m * g * f_r * math.cos(aa) * vx
                     + 0.5 * cwxA * rho * (vx ** 3)
                     + m * g * math.sin(aa) * vx)
            F2.append(F_val)

        absF2 = [abs(x) for x in F2]
        F_min2 = min(absF2)
        F_min_index2 = absF2.index(F_min2)

        try:
            if F_min_index2 == 0 or (F_min_index2 < 10 and abs(F2[F_min_index2 + 1]) < abs(F2[F_min_index2 - 1])):
                alpha_max = alpha2[F_min_index2 + 1]
                alpha_min = alpha2[F_min_index2]
            else:
                alpha_max = alpha2[F_min_index2]
                alpha_min = alpha2[F_min_index2 - 1]
        except:
            alpha_max = alpha2[F_min_index2]
            alpha_min = alpha2[F_min_index2 - 1]

        alpha3 = []
        step_val2 = (alpha_max - alpha_min) / 10.0
        val2 = alpha_min
        for _ in range(11):
            alpha3.append(val2)
            val2 += step_val2

        F3 = []
        for cc in range(11):
            aa = alpha3[cc]
            F_val = (m * g * f_r * math.cos(aa) * vx
                     + 0.5 * cwxA * rho * (vx ** 3)
                     + m * g * math.sin(aa) * vx)
            F3.append(F_val)

        absF3 = [abs(x) for x in F3]
        F_min3 = min(absF3)
        F_min_index3 = absF3.index(F_min3)

        alpha_result = alpha3[F_min_index3]

        return alpha_result

    # Now the main loop that calls Alpha_vx
    for i in range(len(vx_Vector)):
        vx = vx_Vector[i]
        alpha_val = Alpha_vx(m, f_r, vx, cwxA, rho)
        # alpha_Vector(i) = alpha
        alpha_Vector.append(alpha_val)

    return alpha_Vector, vx_Vector
