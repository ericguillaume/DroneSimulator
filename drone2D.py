import numpy as np

class Drone2D:

  # engine 1 is the one on the left, 2 front, 3 right and 4 back in clockwise order
  def __init__(self,engine1,engine2,engine3,engine4):
    self.engine1 = engine1
    self.engine2 = engine2
    self.engine3 = engine3
    self.engine4 = engine4
    self.position = np.zeros((2,1))
    self.matrix_local_to_global = np.eyes(3)
    self.K_pitch_roll = 1.0
    self.K_raw = 0.5
    self.K_force = 2

  def get_accelerations(self):
    speed1 = math.sqrt(self.engine1.get_engine_speed())
    speed2 = math.sqrt(self.engine2.get_engine_speed())
    speed3 = math.sqrt(self.engine3.get_engine_speed())
    speed4 = math.sqrt(self.engine4.get_engine_speed())
    phi = self.K_pitch_roll * (speed1-speed3)
    theta = self.K_pitch_roll * (speed2-speed4)
    psi = self.K_raw * (speed1+speed3-speed2-speed4)
    force = self.K_force * (speed1+speed2+speed3+speed4)
    return phi, theta, psi, force

  def get_matrix_local_to_global(self):
    return matrix_local_to_global

  def update_position(self,dt):
    phi, theta, psi, force = self.get_accelerations()
    cos_psi = math.cos(psi)
    sin_psi = math.sin(psi)
    last_change = np.array([[cos_psi,sin_psi,0],[-sin_psi,cos_psi,0],[0,0,1]])
    self.matrix_local_to_global = np.dot(self.matrix_local_to_global,last_change)
    self.position = self.matrix_local_to_global[0:2,2]


if __name__ == ' __main__':








