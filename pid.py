


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

  def get_command_update_target(self,position,dt,positionD,dpositionD,dPosition = None, debug = False):
    self.update_target(positionD,dpositionD)
    return self.get_command(position,dt,dPosition=dPosition, debug= debug)

  # write test for dPosition = None
  def get_command(self,position,dt,dPosition = None, debug = False):
    if debug:
      print("  target = {}, position = {} ".format(self.positionD, position))
    command = self.Kp*(self.positionD-position) + self.Ki*(self.errorIntegrale)
    if debug:
      print("  Kp term = {}".format(self.Kp*(self.positionD-position)))
      print("  Ki term = {}".format(self.Ki*(self.errorIntegrale)))
    if debug:
      print("  self.lastPosition = {}".format(self.lastPosition))
    if self.lastPosition != None:
      if dPosition == None:
        dPosition = (position-self.lastPosition)/dt
      command += self.Kd*(self.dPositionD - dPosition)
      if debug:
        print("  Kd term = {}, Kd = {}, self.dPositionD = {}, dPosition = {}".format(self.Kd*(self.dPositionD - dPosition), self.Kd, self.dPositionD, dPosition))
    command += self.counterForce
    if debug:
      print("  counter force = {}".format(self.counterForce))
    dt_integral_reduction = dt * self.integral_reduction
    self.errorIntegrale *= (1-dt_integral_reduction)
    if abs(self.positionD-position) < 1.0:
      self.errorIntegrale += (self.positionD-position)*dt  # set the 1 as constant somwzhere else
    # if False and self.name == 'theta':
    #   print(self.name, 'pid')
    #   print('position =', position)
    #   print('positionD =', self.positionD)
    #   print('dPosition =', dPosition)
    #   if self.lastPosition:
    #     print((position-self.lastPosition)/dt)
    #   print('dpositionD =', self.dpositionD)
    #   print('errorIntegrale', self.errorIntegrale + "\n")
    #   print('pid')
    #   print(self.Kp*(self.positionD-position))
    #   print (self.Ki*(self.errorIntegrale))
    #   if self.lastPosition:
    #     print(self.Kd*(self.dPositionD - dPosition))
    #   else:
    #     print('no Kd term')
    #   print('command', command, "\n")
    self.lastPosition = position
    return command











