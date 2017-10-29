from math import asin
import numpy as np

class IMU:

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
    return phi, theta, psi




