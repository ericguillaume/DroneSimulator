from math import asin
import numpy as np


class IMU:

  debug_nb_prints = 0

  def __init__(self):
    self.x = np.array([[1],[0],[0]])
    self.y = np.array([[0],[1],[0]])
    self.z = np.array([[0],[0],[1]])

  def get_orientation_angles(self,matrix_local_to_global):
    x_into_y = float(np.dot(matrix_local_to_global,self.x)[1][0])
    y_into_z = float(np.dot(matrix_local_to_global,self.y)[2][0])
    z_into_x = float(np.dot(matrix_local_to_global,self.z)[0][0])
    phi = asin(y_into_z)
    theta = asin(z_into_x)
    psi = asin(x_into_y)
    if IMU.debug_nb_prints < 100:
      print(phi, theta, psi)
      IMU.debug_nb_prints += 1
    else:
      exit()
    return phi, theta, psi




