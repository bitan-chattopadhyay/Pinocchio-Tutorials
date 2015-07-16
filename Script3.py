# Tutorial 4

from pinocchio.romeo_wrapper import RomeoWrapper
robot = RomeoWrapper('/local/bchattop/devel/src/pinocchio/models/romeo.urdf')
import pinocchio as se3
from pinocchio.utils import *
import numpy as np

execfile("Script1.py")
q = robot.q0
v = rand(robot.nv)

xdes = np.matrix([3.0,1.0,2.0]).T

for i in range(1000):
    Mrh = robot.Mrh(q)
    e = Mrh.translation[0:3,0] - xdes
    J = Mrh.rotation * robot.Jrh(q)[:3,:]
    qdot = -npl.pinv(J)*e
    robot.increment(q,qdot*1e-2)
    robot.display(q)
    updateJointConfiguration(M,'l_wrist')
