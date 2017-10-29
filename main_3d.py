from drone import Drone
import matplotlib.pyplot as plt


class Main:

  def __init__(self):
    self.time_list = []
    self.xy_time_list = []
    self.x_list = []
    self.y_list = []
    self.z_list = []
    self.phi_list = []
    self.theta_list = []
    self.psi_list = []
    self.phiDriver_list = []
    self.thetaDriver_list = []


    self.number_seconds = 60 # 18
    self.max_number_steps = 100
    self.steps_per_second = 100
    self.dt = 1.0/self.steps_per_second

    self.drone = Drone()

  def start(self):
    self.xPositionDesired = 200.0
    self.yPositionDesired = 0.0
    self.zPositionDesired = 0.0
    self.phiD = 0.0 #0.60
    self.thetaD = 0.0 #0.60 # 0.65 works fine for going up as well but we decide to limit at 0.55
    self.d_psiD = 0.0

    for i in range(self.number_seconds):
      for j in range(min(self.max_number_steps-1,self.steps_per_second-1)):
        self.updateDrone(i,j,add_xy_points=False,comment=False)
      self.updateDrone(i,min(self.max_number_steps-1,self.steps_per_second-1),add_xy_points=True,comment=True)

    #plt.plot(self.time_list, self.phi_list, "o")
    #plt.plot(self.time_list, self.theta_list, "o")
    #plt.plot(self.time_list, self.psi_list, "o")
    #self.plot_xy(plt)
    plt.plot(self.xy_time_list, self.x_list, "o")
    #plt.plot(self.xy_time_list, self.y_list, "o")
    #plt.plot(self.time_list, self.z_list, "o")

    #plt.plot(self.time_list, self.phiDriver_list, "o")
    #plt.plot(self.time_list, self.thetaDriver_list, "o")
    plt.show()

  def plot_xy(self,plt):
    plt.plot(self.x_list, self.y_list, "o")
    ax = plt.gca()
    x_lim = ax.get_xlim()
    y_lim = ax.get_ylim()
    min_lim = min(x_lim[0],y_lim[0])
    max_lim = max(x_lim[1],y_lim[1])
    ax.set_xlim(min_lim,max_lim)
    ax.set_ylim(min_lim,max_lim)


  def updateDrone(self,i,j,add_xy_points = False, comment = False):
    position = self.drone.update_position(self.dt,comment=comment)
    phi,theta,psi,phiDriver,thetaDriver = self.drone.update_driver(self.dt,self.xPositionDesired,self.yPositionDesired,self.zPositionDesired)
    self.updateLogs(add_xy_points,float(i*self.steps_per_second+j)/self.steps_per_second,position,phi,theta,psi,phiDriver,thetaDriver)
    if comment:
      #####print 'zCommand =', zCommand-0.5
      #print 'position =', position.T
      print

  def updateLogs(self,add_xy_points,t,position,phi,theta,psi,phiDriver,thetaDriver):
    self.time_list.append(t)
    if add_xy_points:
      self.x_list.append(float(position[0][0]))
      self.y_list.append(float(position[1][0]))
      self.xy_time_list.append(t)
    self.z_list.append(float(position[2][0]))
    self.phi_list.append(phi)
    self.theta_list.append(theta)
    self.psi_list.append(psi)
    self.phiDriver_list.append(phiDriver)
    self.thetaDriver_list.append(thetaDriver)



Main().start()



