# Tutorials 1 and 2

from pinocchio.romeo_wrapper import RomeoWrapper
robot = RomeoWrapper('/local/bchattop/devel/src/pinocchio/models/romeo.urdf')
import pinocchio as se3
from pinocchio.utils import *
import numpy as np

robot.initDisplay("world/romeo/")

def updateJointConfiguration(M,name):
	pinocchioConf = se3ToXYZQUAT(M)
        viewerConf = XYZQUATToViewerConfiguration(pinocchioConf)
        robot.viewer.gui.applyConfiguration('world/romeo/'+name,viewerConf)
	robot.viewer.gui.refresh()

q = robot.q0
qdot = zero(robot.nv)
qdot[20] = 1
dt = 5e-3
for i in range(1000):
    q[7:] += qdot[6:]*dt
    robot.Mrh(q)

qdot[3] = 10
for i in range(100):
    q[3] += qdot[3]*dt
    robot.Mrh(q)

N = 1000
v = zero(3); v[2] = 1.0 / N
w = zero(3); w[1] = 1.0 / N
nu = se3.Motion( v, w )

M = se3.SE3.Identity()
updateJointConfiguration(M,'HeadRollLink')

for i in range(N):
   M = M*se3.exp(nu)
   updateJointConfiguration(M,'HeadRollLink')

N = 1000
Mdes = se3.SE3.Random()
nu = se3.log(M.inverse()*Mdes)

for i in range(N):
    M = M*se3.exp(nu.vector()/N)
    updateJointConfiguration(M,'HeadRollLink')
 
def errorInSE3( M,Mdes):
  '''
    Compute a 6-dim error vector (6x1 np.maptrix) caracterizing the difference
    between M and Mdes, both element of SE3.
  '''
  error = se3.log(M.inverse()*Mdes)
  return error.vector()

N = 1000
Mdes = se3.SE3.Random()
gain = 1.0 / N
for i in range(N):
    nu = gain*errorInSE3(M,Mdes)   # nu is the desired spatial velocity.
    M = M*se3.exp(nu)
    updateJointConfiguration(M,'HeadRollLink') 

def robotint(q,dq):
    M = se3.SE3( se3.Quaternion(q[6,0],q[3,0],q[4,0],q[5,0]).matrix(), q[:3])
    dM = exp(dq[:6])
    M = M*dM
    q[:3] = M.translation
    q[3:7] = se3.Quaternion(M.rotation).coeffs()
    q[7:] += dq[6:]
