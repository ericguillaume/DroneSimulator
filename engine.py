

class Engine():

  def __init__(self):
    self.max_engine_speed = 0.8
    self.set_engine_speed(0.5)

  def get_max_engine_speed(self):
    return self.max_engine_speed

  def get_engine_speed(self):
    return self.engine_speed

  def get_engine_force(self):
    return self.engine_speed * self.engine_speed

  def set_engine_speed(self,value):
    self.engine_speed = value
    if self.engine_speed < 0:
      self.engine_speed = 0
    elif self.engine_speed > self.max_engine_speed:
      self.engine_speed = self.max_engine_speed

