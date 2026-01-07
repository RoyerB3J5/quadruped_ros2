import numpy as np
from .inverse_kinematics import inverse_kinematics_leg

#Limites de las articulaciones en radianes
JOINT_LIMITS = {
  "q1": (-np.deg2rad(90), np.deg2rad(90)),
  "q2": (-np.deg2rad(110), np.deg2rad(60)),
  "q3": (-np.deg2rad(172), -np.deg2rad(24)),
}

#Funcion que aplica el limite
def clamp(value, limits):
  return max(min(value,limits[1]),limits[0])

def solve_leg(x,y,z, leg_name,geometry):
  side = "left" if "left" in leg_name else "right"

  theta1, theta2, theta3 = inverse_kinematics_leg(x,y,z,geometry["L1"], geometry["L2"], geometry["L3"], side)

  theta1 = clamp(theta1, JOINT_LIMITS["q1"])
  theta2 = clamp(theta2, JOINT_LIMITS["q2"])
  theta3 = clamp(theta3, JOINT_LIMITS["q3"])

  return theta1, theta2, theta3