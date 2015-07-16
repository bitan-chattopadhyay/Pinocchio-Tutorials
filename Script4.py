# Tutorial 5

from pinocchio.romeo_wrapper import RomeoWrapper
robot = RomeoWrapper('/local/bchattop/devel/src/pinocchio/models/romeo.urdf')
import pinocchio as se3
from pinocchio.utils import *
import numpy as np

execfile("Script1.py")
q = robot.q0
v = rand(robot.nv)

Mlf = robot.Mlf(q)
z_la_des = Mlf.translation[2,0]

Mrf = robot.Mrf(q)
z_ra_des = Mrf.translation[2,0]

for i in range (1000):
     Mlf=robot.Mlf(q)
     Mrf=robot.Mrh(q)
     xla = Mlf.translation[0,0]
     yla = Mlf.translation[1,0]
     xra = Mrf.translation[0,0]
     yra = Mrf.translation[1,0]
     x_com_des = (xla+xra)/2
     y_com_des = (yla+yra)/2
     Mlf = robot.Mlf(q)
     e_la = Mlf.translation[2,0] - z_la_des
     J_la = Mlf.rotation * robot.Jlf(q)[:3,:]
     J_la = J_la[2,:]
     Mrf = robot.Mrf(q)
     e_ra = Mrf.translation[2,0]-z_ra_des
     J_ra = Mrf.rotation*robot.Jrf(q)[:3,:]
     J_ra = J_ra[2,:]
     Mcom = robot.com(q)
     e_x_com = Mcom[0,0]-x_com_des
     e_y_com = Mcom[1,0]-y_com_des
     Jcom = robot.Jcom(q)[:2,:]
     e = np.matrix([e_la,e_ra,e_x_com,e_y_com]).reshape(4, 1)

     #Append Jacobian
     J = np.matrix([]).reshape(0,39)
     J = np.append(J_la,J_ra,axis=0)
     J = np.append(J,Jcom, axis = 0)
     Jplus = npl.pinv(J)
     qdot = -Jplus*e
     robot.increment(q,qdot*1e-2)
     robot.display(q)
     updateJointConfiguration(M,'l_ankle')
     updateJointConfiguration(M,'r_ankle')
     updateJointConfiguration(M,'body')
