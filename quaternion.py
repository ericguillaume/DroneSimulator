from math import cos, sin
import math


class Quaternion:

  def __init__(self,x,i,j,k):
    self.x = x
    self.i = i
    self.j = j
    self.k = k

  def conjugate(self):
    return Quaternion(self.x,-self.i,-self.j,-self.k)

  def __mul__(self,other):
    new_x = self.x*other.x - self.i*other.i - self.j*other.j - self.k*other.k
    new_i = self.x*other.i + self.i*other.x
    new_i += self.j*other.k - self.k*other.j
    new_j = self.x*other.j + self.j*other.x
    new_j += self.k*other.i - self.i*other.k
    new_k = self.x*other.k + self.k*other.x
    new_k += self.i*other.j - self.j*other.i
    return Quaternion(new_x,new_i,new_j,new_k)

  def __str__(self):
    return str(self.x)+' ('+str(self.i)+','+str(self.j)+','+str(self.k)+')'

unit = Quaternion(1,0,0,0)
i = Quaternion(0,1,0,0)
j = Quaternion(0,0,1,0)
k = Quaternion(0,0,0,1)

theta = math.pi/20

rotation = Quaternion(cos(theta/2),sin(theta/2),0,0)
beforeRotation = Quaternion(0,0,0,1)
afterRotation = rotation*beforeRotation*rotation.conjugate()

print afterRotation


