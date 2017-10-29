from drone import Drone
from pid import PID
import matplotlib.pyplot as plt


def update(i,j,add_xy_points = False, comment = False):
  position = drone.update_position(dt)
  zPosition = float(position[2][0])
  zCommand = pidZ.get_command(zPosition,dt)
  drone.set_engines_speed(zCommand)
  time.append(i+j/100.0)
  z.append(zPosition)
  if add_xy_points:
    x.append(float(position[0][0]))
    y.append(float(position[1][0]))
  if comment:
    #print 'zCommand =', zCommand-0.5
    print 'position =', position.T
    print



time = []
z = []
x = []
y = []
positionDesired = 1.0
derivateDesired = 0.0

dt = 1.0/100
drone = Drone()
pidZ = PID(0.3,0.8,0.008,positionDesired,derivateDesired,counterForce=0.5)  # K=0.3,KD=0.8,Ki=0.001
for i in range(100):
  for j in range(99):
    update(i,j,add_xy_points=False,comment=False)
  update(i,100,add_xy_points=True,comment=True)




plt.plot(time, z, "o")
#plt.plot(x, y, "o")
plt.show()


