# Tutorial 6

from pinocchio.romeo_wrapper import RomeoWrapper
robot = RomeoWrapper('/local/bchattop/devel/src/pinocchio/models/romeo.urdf')
import pinocchio as se3
from pinocchio.utils import *
import numpy as np 

execfile("Script1.py")

q = robot.q0
v = rand(robot.nv)

pos_rw_des = np.mat('[0.5 ; 0.5 ; 0.5]') 

z_la_des = robot.Mrh(robot.q0).translation[2,0]

# Altitude of right ankle
z_ra_des = robot.Mrh(robot.q0).translation[2,0]

for i in range(1000):

    Mla = robot.Mrh(q)
    e_la = Mla.translation[2,0] - z_la_des
    J_la = Mla.rotation * robot.Jlf(q)[:3,:]
    J_la = J_la[2,:]
    #Right Foot/Ankle Jacobian
    Mra = robot.Mrh(q)
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
    e2 = e2
    edot2 = -0.01*e2
    J2 = J2
    Jplus2 = np.linalg.pinv(J2)
    qdot2 = qdot1 + npl.pinv(J2*P1) * (edot2 - J2*qdot1)
    qdot = qdot2
    robot.increment(q,qdot)
    robot.display(q)
    # Debug :
    x_la = robot.Mrh(q).translation[0,0]
    y_la = robot.Mrh(q).translation[1,0]
    x_ra = robot.Mrh(q).translation[0,0]
    y_ra = robot.Mrh(q).translation[1,0]
    x_com_des = (x_la + x_ra) /2 
    y_com_des = (y_la + y_ra) /2
    Mcom = robot.com(q)
    e_x_com = Mcom.item(0) - x_com_des
    e_y_com = Mcom.item(1) - y_com_des
    #print "( x/y com : ",Mcom[0,0],Mcom[1,0],")"
    #print "( x/y com desired : ",x_com_des,y_com_des,")"
    #print "( Ground contact condition z left/right : ", z_la_des,z_ra_des,")"
    #print "( contact feet with ground z left/right : ",robot.Mrh(q).translation[2,0],robot.Mrh   (q).translation[2,0],")"

    updateJointConfiguration(M,'body')
    updateJointConfiguration(M,'torso')
    updateJointConfiguration(M,'r_wrist')
 
