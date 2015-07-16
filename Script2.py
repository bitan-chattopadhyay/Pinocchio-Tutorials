# Tutorial 3

import pinocchio as se3
from pinocchio.utils import *
import numpy as np

execfile("Script1.py")
q = robot.q0
v = rand(robot.nv)

xdes = 3.0

for i in range(1000):
    Mrh = robot.Mrh(q)
    e = Mrh.translation[0,0] - xdes
    J = Mrh.rotation * robot.Jrh(q)[:3,:]
    robot.increment(q,qdot*1e-2)
    robot.display(q)
    updateJointConfiguration(M,'r_wrist')
