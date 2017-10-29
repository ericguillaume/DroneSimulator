from engine import Engine
from imu import IMU
from attitude_controller import AttitudeController
from pid import PID
import math
import numpy as np

class Drone:

  # engine 1 is the one on the left, 2 front, 3 right and 4 back in clockwise order
  def __init__(self):
    #self.matrix_local_to_global = np.eye(3)
    startCos = math.cos(0.0)#0.5)
    startSin = math.sin(0.0)#0.5)
    self.matrix_local_to_global = np.array([[startCos,0,startSin],[0,1,0],[-startSin,0,startCos]])
    self.position = np.zeros((3,1))
    self.d_position = np.zeros((3,1)) # dx, dy, dz
    self.phi = 0.0
    self.theta = 0.0
    self.psi = 0.0
    self.d_phi = 0.0
    self.d_theta = 0.0
    self.d_psi = 0.0

    self.G_pitch_roll = 0.5
    self.G_raw = 0.1
    self.G_force = 1

    self.weight = 1.0
    self.kPsiRotationResistance = 0.1
    self.kAirResistance = 0.05

    self.engine1 = Engine()
    self.engine2 = Engine()
    self.engine3 = Engine()
    self.engine4 = Engine()
    self.engines = [self.engine1,self.engine2,self.engine3,self.engine4]
    self.imu = IMU()

    self.zKp = 0.01
    self.zKi = 0.01#0.005
    self.zKd = 0.0#0.5
    self.phiThetaKp = 0.2
    self.phiThetaKi = 0.001
    self.phiThetaKd = 0.5
    self.psiKp = 20.0
    self.psiKi = 0.5
    self.psiKd = 0.1 # on envoit une ligne plus pas du dip   alors que PID recoid PDI ?? retc...
    self.attitude_controller = AttitudeController(self.zKp,self.zKd,self.zKi,self.phiThetaKp,self.phiThetaKi,self.phiThetaKd,self.psiKp,self.psiKi,self.psiKd)# move those parameters as configurations parameters

    self.phiThetaDriverKd = 2.0
    self.phiThetaDriverKp = self.phiThetaDriverKd / 200
    self.phiThetaDriverKi = 0.0
    self.phiDriverPID = PID('phi_driver',self.phiThetaDriverKp,self.phiThetaDriverKi,self.phiThetaDriverKd,0,dpositionD=0,counterForce=0)
    self.thetaDriverPID = PID('theta_driver',self.phiThetaDriverKp,self.phiThetaDriverKi,self.phiThetaDriverKd,0,dpositionD=0,counterForce=0)

  def get_accelerations(self):
    force1 = self.engine1.get_engine_force()
    force2 = self.engine2.get_engine_force()
    force3 = self.engine3.get_engine_force()
    force4 = self.engine4.get_engine_force()
    #print 'engines forces',force1,force2,force3,force4
    d2_phi = self.G_pitch_roll * (force1-force3)
    d2_theta = -self.G_pitch_roll * (force2-force4)
    d2_psi = self.G_raw * (force1+force3-force2-force4) #- self.kPsiRotationResistance*self.d_psi#*self.d_psi
    zforce_localc = self.G_force * (force1+force2+force3+force4)
    #print 'forces phi theta psi', d2_phi, d2_theta, d2_psi
    return d2_phi, d2_theta, d2_psi, zforce_localc

  def set_engines_speed(self,value):
    for engine in self.engines:
      engine.set_engine_speed(value)

  def get_matrix_local_to_global(self):
    return matrix_local_to_global

  def update_position(self,dt,comment=False):
    d2_phi, d2_theta, d2_psi, zforce_localc = self.get_accelerations()
    #d2_theta += 0.1  # 0.0005 small change    # for psi we prooved we can resist up to 0.02 and for theta/psi up to 0.2

    cos_d2phi = math.cos(self.d_phi*dt)
    sin_d2phi = math.sin(self.d_phi*dt)
    changePhi = np.array([[1,0,0],[0,cos_d2phi,-sin_d2phi],[0,sin_d2phi,cos_d2phi]])
    cos_d2theta = math.cos(self.d_theta*dt)
    sin_d2theta = math.sin(self.d_theta*dt)
    changeTheta = np.array([[cos_d2theta,0,sin_d2theta],[0,1,0],[-sin_d2theta,0,cos_d2theta]])
    cos_d2psi = math.cos(self.d_psi*dt)
    sin_d2psi = math.sin(self.d_psi*dt)
    changePsi = np.array([[cos_d2psi,-sin_d2psi,0],[sin_d2psi,cos_d2psi,0],[0,0,1]])

    force_globalc = np.dot(self.matrix_local_to_global,np.array([[0],[0],[zforce_localc]]))
    force_globalc = force_globalc - self.kAirResistance * self.d_position
    force_globalc[2] = force_globalc[2] - self.weight

    self.matrix_local_to_global = np.dot(self.matrix_local_to_global,changePhi)
    self.matrix_local_to_global = np.dot(self.matrix_local_to_global,changeTheta)
    self.matrix_local_to_global = np.dot(self.matrix_local_to_global,changePsi)
    if comment:
      print(self.matrix_local_to_global, "\n")

    self.position = self.position + (dt * self.d_position)
    self.d_position = self.d_position + (dt * force_globalc)
    self.phi += self.d_phi*dt # wouldn't it be more precise to use IMU and the matrix
    self.theta += self.d_theta *dt
    self.psi += self.d_psi*dt
    self.d_phi += d2_phi*dt
    self.d_theta += d2_theta*dt
    self.d_psi += d2_psi*dt
    return self.position

  def update_attitude(self,dt,zD,phiD,thetaD,d_psiD):
    z = float(self.position[2][0])
    phi,theta,psi = self.imu.get_orientation_angles(self.matrix_local_to_global)
    engine1_control,engine2_control,engine3_control,engine4_control = self.attitude_controller.get_engines_controls(self.engine1.get_max_engine_speed(),dt,z,zD,phi,self.d_phi,phiD,theta,self.d_theta,thetaD,self.d_psi,d_psiD)
    self.engine1.set_engine_speed(engine1_control)
    self.engine2.set_engine_speed(engine2_control)
    self.engine3.set_engine_speed(engine3_control)
    self.engine4.set_engine_speed(engine4_control)
    return phi,theta,psi

  def update_driver(self,dt,xD,yD,zD): #LATER On it will be separated by timing from update_attitude
    globalPositionD = np.array([[xD],[yD],[1]])
    cos_psi = math.cos(self.psi)
    sin_psi = math.sin(self.psi)
    psiRotation = np.array([[cos_psi,-sin_psi],[sin_psi,cos_psi]])
    reverseTranslation = -np.dot(psiRotation,self.position[0:2])
    matrix_global_to_local_psi = np.array([[cos_psi,sin_psi,reverseTranslation[0]],[-sin_psi,cos_psi,reverseTranslation[1]],[0,0,1]])
    localPositionD = np.dot(matrix_global_to_local_psi,globalPositionD)

    xPositionD = float(localPositionD[0])
    yPositionD = float(localPositionD[1])
    if xPositionD > math.sin(0.55)/math.cos(0.55)*1.0/self.kAirResistance/self.kAirResistance:
      thetaD = 0.55
    else:
      thetaD = self.thetaDriverPID.get_command_update_target(0.0,dt,xPositionD,0,dPosition=self.theta)
      print(xPositionD, self.theta, thetaD)
    phiD = -self.phiDriverPID.get_command_update_target(0.0,dt,yPositionD,0,dPosition=None)

    # put aside this saturation ??
    thetaD = max(min(thetaD,0.55),-0.55) # put aside 0.55 in constant
    phiD = max(min(phiD,0.55),-0.55) # put aside 0.55 in constant

    d_psiD=0# set this value later on

    phi,theta,psi = self.update_attitude(dt,zD,phiD,thetaD,d_psiD)
    return phi,theta,psi,phiD,thetaD






