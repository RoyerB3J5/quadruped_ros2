import numpy as np
from robot_kinematics.leg_solver import solve_leg

#Distancia de los eslabones en metros
geometry = {
  "L1": 0.04005,
  "L2": 0.09,
  "L3": 0.09,
}

angles = solve_leg(x=0.0,y=-0.05,z=0.0,leg_name="front_left",geometry=geometry)

print("Angulos en grados:")
print(np.rad2deg(angles))
