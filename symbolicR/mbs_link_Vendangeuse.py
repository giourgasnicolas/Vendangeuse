#
#	MBsysTran - Release 8.1
#
#	Copyright 
#	Universite catholique de Louvain (UCLouvain) 
#	Mechatronic, Electrical Energy, and Dynamic systems (MEED Division) 
#	2, Place du Levant
#	1348 Louvain-la-Neuve 
#	Belgium 
#
#	http://www.robotran.be 
#
#	==> Generation Date: Sat Mar  5 18:19:52 2022
#
#	==> Project name: Vendangeuse
#
#	==> Number of joints: 22
#
#	==> Function: F7 - Link Forces (1D)
#
#	==> Git hash: 220d76a94da8547597795a13c9921f0375de3fc1
#

from math import sin, cos, sqrt

def link(frc, trq, Flink, Z, Zd, s, tsim):
    q = s.q
    qd = s.qd
 
# Trigonometric functions

    S18 = sin(q[18])
    C18 = cos(q[18])
    S7 = sin(q[7])
    C7 = cos(q[7])
    S8 = sin(q[8])
    C8 = cos(q[8])
    S19 = sin(q[19])
    C19 = cos(q[19])
 
# Augmented Joint Position Vectors

 
# Link Kinematics: displacement (Z) , relative velocity (Zd)

    RLlnk2_22 = s.dpt[2,30]*C18-s.dpt[3,30]*S18
    RLlnk2_32 = s.dpt[2,30]*S18+s.dpt[3,30]*C18
    POlnk2_22 = RLlnk2_22+s.dpt[2,8]
    ORlnk2_22 = -qd[18]*RLlnk2_32
    ORlnk2_32 = qd[18]*RLlnk2_22
    Plnk21 = POlnk2_22-s.dpt[2,5]
    Plnk31 = RLlnk2_32-s.dpt[3,5]
    PPlnk1 = Plnk21*Plnk21+Plnk31*Plnk31
    Z1 = sqrt(PPlnk1)
    e21 = Plnk21/Z1
    e31 = Plnk31/Z1
    Zd1 = ORlnk2_22*e21+ORlnk2_32*e31
    RLlnk4_22 = s.dpt[2,13]*C7-s.dpt[3,13]*S7
    RLlnk4_32 = s.dpt[2,13]*S7+s.dpt[3,13]*C7
    POlnk4_22 = RLlnk4_22+s.dpt[2,1]
    ORlnk4_22 = -qd[7]*RLlnk4_32
    ORlnk4_32 = qd[7]*RLlnk4_22
    Plnk22 = POlnk4_22-s.dpt[2,6]
    Plnk32 = RLlnk4_32-s.dpt[3,6]
    PPlnk2 = Plnk22*Plnk22+Plnk32*Plnk32
    Z2 = sqrt(PPlnk2)
    e22 = Plnk22/Z2
    e32 = Plnk32/Z2
    Zd2 = ORlnk4_22*e22+ORlnk4_32*e32
    RLlnk6_22 = s.dpt[2,15]*C8-s.dpt[3,15]*S8
    RLlnk6_32 = s.dpt[2,15]*S8+s.dpt[3,15]*C8
    POlnk6_22 = RLlnk6_22+s.dpt[2,2]
    ORlnk6_22 = -qd[8]*RLlnk6_32
    ORlnk6_32 = qd[8]*RLlnk6_22
    Plnk13 = -s.dpt[1,10]+s.dpt[1,2]
    Plnk23 = POlnk6_22-s.dpt[2,10]
    Plnk33 = RLlnk6_32-s.dpt[3,10]
    PPlnk3 = Plnk13*Plnk13+Plnk23*Plnk23+Plnk33*Plnk33
    Z3 = sqrt(PPlnk3)
    e13 = Plnk13/Z3
    e23 = Plnk23/Z3
    e33 = Plnk33/Z3
    Zd3 = ORlnk6_22*e23+ORlnk6_32*e33
    RLlnk8_22 = s.dpt[2,32]*C19-s.dpt[3,32]*S19
    RLlnk8_32 = s.dpt[2,32]*S19+s.dpt[3,32]*C19
    POlnk8_22 = RLlnk8_22+s.dpt[2,9]
    ORlnk8_22 = -qd[19]*RLlnk8_32
    ORlnk8_32 = qd[19]*RLlnk8_22
    Plnk14 = -s.dpt[1,12]+s.dpt[1,9]
    Plnk24 = POlnk8_22-s.dpt[2,12]
    Plnk34 = RLlnk8_32-s.dpt[3,12]
    PPlnk4 = Plnk14*Plnk14+Plnk24*Plnk24+Plnk34*Plnk34
    Z4 = sqrt(PPlnk4)
    e14 = Plnk14/Z4
    e24 = Plnk24/Z4
    e34 = Plnk34/Z4
    Zd4 = ORlnk8_22*e24+ORlnk8_32*e34

# Link Forces 

    Flink1 = s.user_LinkForces(Z1,Zd1,s,tsim,1)
    Flink2 = s.user_LinkForces(Z2,Zd2,s,tsim,2)
    Flink3 = s.user_LinkForces(Z3,Zd3,s,tsim,3)
    Flink4 = s.user_LinkForces(Z4,Zd4,s,tsim,4)
 
# Link Dynamics: forces projection on body-fixed frames

    fPlnk21 = Flink1*e21
    fPlnk31 = Flink1*e31
    trqlnk6_1_1 = -fPlnk21*(s.dpt[3,5]-s.l[3,6])+fPlnk31*s.dpt[2,5]
    trqlnk6_1_2 = fPlnk31*s.l[1,6]
    trqlnk6_1_3 = -fPlnk21*s.l[1,6]
    fSlnk21 = Flink1*(e21*C18+e31*S18)
    fSlnk31 = Flink1*(-e21*S18+e31*C18)
    trqlnk18_1_1 = fSlnk21*s.dpt[3,30]-fSlnk31*(s.dpt[2,30]-s.l[2,18])
    fPlnk22 = Flink2*e22
    fPlnk32 = Flink2*e32
    frclnk6_2_2 = fPlnk21+fPlnk22
    frclnk6_2_3 = fPlnk31+fPlnk32
    trqlnk6_2_1 = trqlnk6_1_1-fPlnk22*(s.dpt[3,6]-s.l[3,6])+fPlnk32*s.dpt[2,6]
    trqlnk6_2_2 = trqlnk6_1_2+fPlnk32*s.l[1,6]
    trqlnk6_2_3 = trqlnk6_1_3-fPlnk22*s.l[1,6]
    fSlnk22 = Flink2*(e22*C7+e32*S7)
    fSlnk32 = Flink2*(-e22*S7+e32*C7)
    trqlnk7_2_1 = fSlnk22*s.dpt[3,13]-fSlnk32*(s.dpt[2,13]-s.l[2,7])
    fPlnk13 = Flink3*e13
    fPlnk23 = Flink3*e23
    fPlnk33 = Flink3*e33
    frclnk6_3_2 = fPlnk23+frclnk6_2_2
    frclnk6_3_3 = fPlnk33+frclnk6_2_3
    trqlnk6_3_1 = trqlnk6_2_1-fPlnk23*(s.dpt[3,10]-s.l[3,6])+fPlnk33*s.dpt[2,10]
    trqlnk6_3_2 = trqlnk6_2_2+fPlnk13*(s.dpt[3,10]-s.l[3,6])-fPlnk33*(s.dpt[1,10]-s.l[1,6])
    trqlnk6_3_3 = trqlnk6_2_3-fPlnk13*s.dpt[2,10]+fPlnk23*(s.dpt[1,10]-s.l[1,6])
    fSlnk13 = Flink3*e13
    fSlnk23 = Flink3*(e23*C8+e33*S8)
    fSlnk33 = Flink3*(-e23*S8+e33*C8)
    trqlnk8_3_1 = fSlnk23*s.dpt[3,15]-fSlnk33*(s.dpt[2,15]-s.l[2,8])
    trqlnk8_3_2 = -fSlnk13*s.dpt[3,15]
    trqlnk8_3_3 = fSlnk13*(s.dpt[2,15]-s.l[2,8])
    fPlnk14 = Flink4*e14
    fPlnk24 = Flink4*e24
    fPlnk34 = Flink4*e34
    frclnk6_4_1 = fPlnk13+fPlnk14
    frclnk6_4_2 = fPlnk24+frclnk6_3_2
    frclnk6_4_3 = fPlnk34+frclnk6_3_3
    trqlnk6_4_1 = trqlnk6_3_1-fPlnk24*(s.dpt[3,12]-s.l[3,6])+fPlnk34*s.dpt[2,12]
    trqlnk6_4_2 = trqlnk6_3_2+fPlnk14*(s.dpt[3,12]-s.l[3,6])-fPlnk34*(s.dpt[1,12]-s.l[1,6])
    trqlnk6_4_3 = trqlnk6_3_3-fPlnk14*s.dpt[2,12]+fPlnk24*(s.dpt[1,12]-s.l[1,6])
    fSlnk14 = Flink4*e14
    fSlnk24 = Flink4*(e24*C19+e34*S19)
    fSlnk34 = Flink4*(-e24*S19+e34*C19)
    trqlnk19_4_1 = fSlnk24*s.dpt[3,32]-fSlnk34*(s.dpt[2,32]-s.l[2,19])
    trqlnk19_4_2 = -fSlnk14*s.dpt[3,32]
    trqlnk19_4_3 = fSlnk14*(s.dpt[2,32]-s.l[2,19])
 
# Symbolic model output

    frc[1,6] = s.frc[1,6]+frclnk6_4_1
    frc[2,6] = s.frc[2,6]+frclnk6_4_2
    frc[3,6] = s.frc[3,6]+frclnk6_4_3
    trq[1,6] = s.trq[1,6]+trqlnk6_4_1
    trq[2,6] = s.trq[2,6]+trqlnk6_4_2
    trq[3,6] = s.trq[3,6]+trqlnk6_4_3
    frc[2,7] = s.frc[2,7]-fSlnk22
    frc[3,7] = s.frc[3,7]-fSlnk32
    trq[1,7] = s.trq[1,7]+trqlnk7_2_1
    frc[1,8] = s.frc[1,8]-fSlnk13
    frc[2,8] = s.frc[2,8]-fSlnk23
    frc[3,8] = s.frc[3,8]-fSlnk33
    trq[1,8] = s.trq[1,8]+trqlnk8_3_1
    trq[2,8] = s.trq[2,8]+trqlnk8_3_2
    trq[3,8] = s.trq[3,8]+trqlnk8_3_3
    frc[2,18] = s.frc[2,18]-fSlnk21
    frc[3,18] = s.frc[3,18]-fSlnk31
    trq[1,18] = s.trq[1,18]+trqlnk18_1_1
    frc[1,19] = s.frc[1,19]-fSlnk14
    frc[2,19] = s.frc[2,19]-fSlnk24
    frc[3,19] = s.frc[3,19]-fSlnk34
    trq[1,19] = s.trq[1,19]+trqlnk19_4_1
    trq[2,19] = s.trq[2,19]+trqlnk19_4_2
    trq[3,19] = s.trq[3,19]+trqlnk19_4_3
 
# Symbolic model output

    Z[1] = Z1
    Zd[1] = Zd1
    Flink[1] = Flink1
    Z[2] = Z2
    Zd[2] = Zd2
    Flink[2] = Flink2
    Z[3] = Z3
    Zd[3] = Zd3
    Flink[3] = Flink3
    Z[4] = Z4
    Zd[4] = Zd4
    Flink[4] = Flink4

# Number of continuation lines = 0


