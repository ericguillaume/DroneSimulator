

class PID:

  # we receive in order pdi instead of pid here
  def __init__(self,name,Kp,Kd,Ki,positionD,dpositionD=0,counterForce=0):
    self.integral_reduction = 0.035 # put somwhere else  # 0.035
    self.name = name
    self.Kp = Kp
    self.Kd = Kd
    self.Ki = Ki
    self.positionD = positionD # position desired
    self.dPositionD = dpositionD
    self.errorIntegrale = 0
    self.lastPosition = None
    self.counterForce = counterForce

  def update_target(self,positionD,dpositionD):
    self.positionD = positionD
    self.dpositionD = dpositionD

  def get_command_update_target(self,position,dt,positionD,dpositionD,dPosition = None):
    self.update_target(positionD,dpositionD)
    return self.get_command(position,dt,dPosition=dPosition)

  # write test for dPosition = None
  def get_command(self,position,dt,dPosition = None):
    command = self.Kp*(self.positionD-position) + self.Ki*(self.errorIntegrale)
    if self.lastPosition:
      if dPosition == None:
        dPosition = (position-self.lastPosition)/dt
      command += self.Kd*(self.dPositionD - dPosition)
    command += self.counterForce
    dt_integral_reduction = dt * self.integral_reduction
    self.errorIntegrale *= (1-dt_integral_reduction)
    if abs(self.positionD-position) < 1.0:
      self.errorIntegrale += (self.positionD-position)*dt  # set the 1 as constant somwzhere else
    if False and self.name == 'theta':
      print(self.name, 'pid')
      print('position =', position)
      print('positionD =', self.positionD)
      print('dPosition =', dPosition)
      if self.lastPosition:
        print((position-self.lastPosition)/dt)
      print('dpositionD =', self.dpositionD)
      print('errorIntegrale', self.errorIntegrale + "\n")
      print('pid')
      print(self.Kp*(self.positionD-position))
      print (self.Ki*(self.errorIntegrale))
      if self.lastPosition:
        print(self.Kd*(self.dPositionD - dPosition))
      else:
        print('no Kd term')
      print('command', command, "\n")
    self.lastPosition = position
    return command











