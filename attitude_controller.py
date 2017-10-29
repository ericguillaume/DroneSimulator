from pid import PID


class AttitudeController:

  def __init__(self,zKp,zKi,zKd,phiThetaKp,phiThetaKi,phiThetaKd,psiKp,psiKi,psiKd):
    # counter force shouold be an argument given
    self.zPID = PID('z',zKp,zKd,zKi,0,dpositionD=0,counterForce=0.5) # change its gain K too
    self.phiPID = PID('phi',phiThetaKp,phiThetaKd,phiThetaKi,0,dpositionD=0,counterForce=0)
    self.thetaPID = PID('theta',phiThetaKp,phiThetaKd,phiThetaKi,0,dpositionD=0,counterForce=0)
    self.dPsiPID = PID('psi',psiKp,psiKd,psiKi,0,dpositionD=0,counterForce=0) # separete the coefficients from above

  # should we give those the d_theta or let them compute them ??
  def get_attitude_controls(self,dt,z,zD,phi,d_phi,phiD,theta,d_theta,thetaD,d_psi,d_psiD):
    z_command = self.zPID.get_command_update_target(z,dt,zD,0,dPosition=None)
    z_command = max(z_command,0)
    phi_command = self.phiPID.get_command_update_target(phi,dt,phiD,0,dPosition=d_phi)
    theta_command = self.thetaPID.get_command_update_target(theta,dt,thetaD,0,dPosition=d_theta)
    dpsi_command = self.dPsiPID.get_command_update_target(d_psi,dt,d_psiD,0,dPosition=None)
    #print 'commands z,phi,theta,psi',z_command, phi_command, theta_command, dpsi_command
    return z_command, phi_command, theta_command, dpsi_command

  def get_engines_controls(self,engine_max_speed,dt,z,zD,phi,d_phi,phiD,theta,d_theta,thetaD,d_psi,d_psiD):
    z_command, phi_command, theta_command, dpsi_command = self.get_attitude_controls(dt,z,zD,phi,d_phi,phiD,theta,d_theta,thetaD,d_psi,d_psiD)
    engine1_control = z_command+phi_command+dpsi_command
    engine2_control = z_command-theta_command-dpsi_command
    engine3_control = z_command-phi_command+dpsi_command
    engine4_control = z_command+theta_command-dpsi_command
    return self.adapt_engines_controls_to_saturation(engine_max_speed,engine1_control,engine2_control,engine3_control,engine4_control)

  def adapt_engines_controls_to_saturation(self,engine_max_speed,engine1_control,engine2_control,engine3_control,engine4_control):
    mim_engine_control = min(engine1_control,engine2_control)
    mim_engine_control = min(mim_engine_control,engine3_control)
    mim_engine_control = min(mim_engine_control,engine4_control)
    negative_value_of_min_engine_control = -min(mim_engine_control,0)
    engine1_control += negative_value_of_min_engine_control
    engine2_control += negative_value_of_min_engine_control
    engine3_control += negative_value_of_min_engine_control
    engine4_control += negative_value_of_min_engine_control

    max_engine_control = max(engine1_control,engine2_control)
    max_engine_control = max(max_engine_control,engine3_control)
    max_engine_control = max(max_engine_control,engine4_control)
    if max_engine_control:
      coefficient = min(engine_max_speed,max_engine_control)/max_engine_control
      engine1_control *= coefficient
      engine2_control *= coefficient
      engine3_control *= coefficient
      engine4_control *= coefficient
    #print 'engines_controls',engine1_control,engine2_control,engine3_control,engine4_control
    return engine1_control,engine2_control,engine3_control,engine4_control



