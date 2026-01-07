import numpy as np
JOINT_LIMITS = {
    "theta1": (-1.57,1.57),
    "theta2": (-1.0472,1.885),
    "theta3": (-3.0019,-0.4189)
}

def clamp(angle, mn, mx):
    return max(min(angle,mx),mn)
    
def inverse_kinematics_leg(x4, y4, z4, L1, L2, L3, leg_side):
    """
    Cinemática inversa para una pata del robot cuadrúpedo.

    Convención:
    - Sentido antihorario positivo
    - Rodilla hacia atrás => q3 < 0
    - Flexión de cadera hacia adelante => q2 > 0
    """

    D = (y4**2 + z4**2 - L1**2 + x4**2 - L2**2 - L3**2) / (2 * L2 * L3)

    if abs(D) > 1.0:
        raise ValueError("Posición fuera de alcance")

    theta3 = np.arctan2(-np.sqrt(1 - D**2), D)

    radial = y4**2 + z4**2 - L1**2
    if radial < 0:
        raise ValueError("Posicion no valida:")

    r = np.sqrt(radial)

    y_term = x4
    x_term = r

    term2_y = L3 * np.sin(theta3)
    term2_x = L2 + L3 * np.cos(theta3)

    theta2 = -np.arctan2(y_term, x_term) + np.arctan2(term2_y, term2_x)

    theta2 = -theta2

    # ---- Selección de pata ----
    if leg_side == "left":
        theta1 = -np.arctan2(-z4, y4) - np.arctan2(r, -L1)
        theta1 += np.pi

    elif leg_side == "right":
        theta1 = -np.arctan2(z4, y4) - np.arctan2(r, -L1)

    else:
        raise ValueError("leg_side debe ser 'left' o 'right'")
    
    theta1 = clamp(theta1, *JOINT_LIMITS["theta1"])
    theta2 = clamp(theta2, *JOINT_LIMITS["theta2"])
    theta3 = clamp(theta3, *JOINT_LIMITS["theta3"])
    
    return theta1, theta2, theta3
