# Tutorial 7

from pinocchio.romeo_wrapper import RomeoWrapper
robot = RomeoWrapper('/local/bchattop/devel/src/pinocchio/models/romeo.urdf')
import pinocchio as se3
from pinocchio.utils import *
import numpy as np

execfile("Script1.py")

q = robot.q0
v = rand(robot.nv)

pos_rw_des = np.mat('[0.5 ; 0.5 ; 0.5]') 
q_post_des = robot.q0 

z_la_des = robot.Mrh(robot.q0).translation[2,0]

# Altitude of right ankle
z_ra_des = robot.Mrh(robot.q0).translation[2,0]

for i in range(1000):
 
    Mla = robot.Mrf(q)
    e_la = Mla.translation[2,0] - z_la_des
    J_la = Mla.rotation * robot.Jlf(q)[:3,:]
    J_la = J_la[2,:]
    #Right Foot/Ankle Jacobian
    Mra = robot.Mrf(q)
    e_ra = Mra.translation[2,0] - z_ra_des
    J_ra = Mra.rotation * robot.Jrf(q)[:3,:]
    J_ra = J_ra[2,:]
    e1 = np.matrix([e_la,e_ra]).reshape (2, 1) 
    J1 = np.matrix([]).reshape (0, 39)
    J1 = np.append(J_la,J_ra, axis = 0)
    edot1 = -0.01*e1
    Jplus1 = np.linalg.pinv(J1)
    P1 = eye(robot.nq - 1) - Jplus1*J1
    qdot1 = Jplus1 * edot1
    Mrh = robot.Mrh(q)
    e2 = Mrh.translation[0:3,0] - pos_rw_des
    J2 = Mrh.rotation * robot.Jrh(q)[:3,:]
    Jplus2 = npl.pinv(J2)
    edot2 = -0.01*e2
    Jplus2 = np.linalg.pinv(J2)
    P2 = P1 - np.linalg.pinv(J2*P1)*J2*P1
    qdot2 = qdot1 + np.linalg.pinv(J2*P1) * (edot2 - J2*qdot1)
    e3 = q - q_post_des
    J3 = eye(robot.nv)
    Jplus3 = npl.pinv(J3)
    edot3 = -0.01*e3
    edot3_reduced = np.delete(edot3,6)
    qdot3 = (qdot2 + npl.pinv(J3*P2) * (edot3_reduced - J3*qdot2))
    robot.increment(q,qdot2)
    robot.display(q)
    updateJointConfiguration(M,'body')
    updateJointConfiguration(M,'torso')

