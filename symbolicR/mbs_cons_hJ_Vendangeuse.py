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
#	==> Generation Date: Mon May  2 16:00:06 2022
#
#	==> Project name: Vendangeuse
#
#	==> Number of joints: 31
#
#	==> Function: F8 - Constraints and Constraints Jacobian(h, J)
#
#	==> Git hash: 220d76a94da8547597795a13c9921f0375de3fc1
#

from math import sin, cos

def cons_hJ(h, Jac, s):
    q = s.q
 
# Trigonometric functions

    S29 = sin(q[29])
    C29 = cos(q[29])
    S7 = sin(q[7])
    C7 = cos(q[7])
    S8 = sin(q[8])
    C8 = cos(q[8])
    S9 = sin(q[9])
    C9 = cos(q[9])
    S18 = sin(q[18])
    C18 = cos(q[18])
    S19 = sin(q[19])
    C19 = cos(q[19])
    S20 = sin(q[20])
    C20 = cos(q[20])
 
# Augmented Joint Position Vectors

 
# Constraints and Constraints Jacobian

    RLlp1_12 = s.dpt[1,18]*C29-s.dpt[2,18]*S29
    RLlp1_22 = s.dpt[1,18]*S29+s.dpt[2,18]*C29
    ROlp2_22 = S7*S8
    ROlp2_32 = -C7*S8
    ROlp2_82 = -S7*C8
    ROlp2_92 = C7*C8
    ROlp2_13 = C8*C9
    ROlp2_23 = ROlp2_22*C9+C7*S9
    ROlp2_33 = ROlp2_32*C9+S7*S9
    RLlp2_14 = q[10]*S8
    RLlp2_24 = ROlp2_82*q[10]
    RLlp2_34 = ROlp2_92*q[10]
    POlp2_14 = RLlp2_14+s.dpt[1,1]
    POlp2_24 = RLlp2_24+s.dpt[2,1]
    POlp2_34 = RLlp2_34+s.dpt[3,1]
    JTlp2_14_2 = -RLlp2_24*S7+RLlp2_34*C7
    JTlp2_24_2 = RLlp2_14*S7
    JTlp2_34_2 = -RLlp2_14*C7
    JTlp2_14_3 = -RLlp2_24*ROlp2_92+RLlp2_34*ROlp2_82
    JTlp2_24_3 = RLlp2_14*ROlp2_92-RLlp2_34*S8
    JTlp2_34_3 = -RLlp2_14*ROlp2_82+RLlp2_24*S8
    RLlp2_15 = ROlp2_13*s.dpt[1,8]+s.dpt[3,8]*S8
    RLlp2_25 = ROlp2_23*s.dpt[1,8]+ROlp2_82*s.dpt[3,8]
    RLlp2_35 = ROlp2_33*s.dpt[1,8]+ROlp2_92*s.dpt[3,8]
    POlp2_15 = POlp2_14+RLlp2_15
    POlp2_25 = POlp2_24+RLlp2_25
    POlp2_35 = POlp2_34+RLlp2_35
    JTlp2_25_1 = -RLlp2_34-RLlp2_35
    JTlp2_35_1 = RLlp2_24+RLlp2_25
    JTlp2_15_2 = JTlp2_14_2-RLlp2_25*S7+RLlp2_35*C7
    JTlp2_25_2 = JTlp2_24_2+RLlp2_15*S7
    JTlp2_35_2 = JTlp2_34_2-RLlp2_15*C7
    JTlp2_15_3 = JTlp2_14_3-RLlp2_25*ROlp2_92+RLlp2_35*ROlp2_82
    JTlp2_25_3 = JTlp2_24_3+RLlp2_15*ROlp2_92-RLlp2_35*S8
    JTlp2_35_3 = JTlp2_34_3-RLlp2_15*ROlp2_82+RLlp2_25*S8
    Plp11 = -POlp2_15+RLlp1_12
    Plp21 = -POlp2_25+RLlp1_22
    Plp31 = -POlp2_35+s.dpt[3,5]
    P2lp1 = Plp11*Plp11+Plp21*Plp21+Plp31*Plp31
    l2rod1 = s.lrod[1]*s.lrod[1]
    h_1 = (0.50)*(P2lp1-l2rod1)
    Jac_1_29 = -Plp11*RLlp1_22+Plp21*RLlp1_12
    Jac_1_7 = -JTlp2_25_1*Plp21-JTlp2_35_1*Plp31
    Jac_1_8 = -JTlp2_15_2*Plp11-JTlp2_25_2*Plp21-JTlp2_35_2*Plp31
    Jac_1_9 = -JTlp2_15_3*Plp11-JTlp2_25_3*Plp21-JTlp2_35_3*Plp31
    Jac_1_10 = -Plp11*S8-Plp21*ROlp2_82-Plp31*ROlp2_92
    RLlp3_12 = s.dpt[1,17]*C29-s.dpt[2,17]*S29
    RLlp3_22 = s.dpt[1,17]*S29+s.dpt[2,17]*C29
    ROlp4_22 = S18*S19
    ROlp4_32 = -C18*S19
    ROlp4_82 = -S18*C19
    ROlp4_92 = C18*C19
    ROlp4_13 = C19*C20
    ROlp4_23 = ROlp4_22*C20+C18*S20
    ROlp4_33 = ROlp4_32*C20+S18*S20
    RLlp4_14 = q[21]*S19
    RLlp4_24 = ROlp4_82*q[21]
    RLlp4_34 = ROlp4_92*q[21]
    POlp4_14 = RLlp4_14+s.dpt[1,3]
    POlp4_24 = RLlp4_24+s.dpt[2,3]
    POlp4_34 = RLlp4_34+s.dpt[3,3]
    JTlp4_14_2 = -RLlp4_24*S18+RLlp4_34*C18
    JTlp4_24_2 = RLlp4_14*S18
    JTlp4_34_2 = -RLlp4_14*C18
    JTlp4_14_3 = -RLlp4_24*ROlp4_92+RLlp4_34*ROlp4_82
    JTlp4_24_3 = RLlp4_14*ROlp4_92-RLlp4_34*S19
    JTlp4_34_3 = -RLlp4_14*ROlp4_82+RLlp4_24*S19
    RLlp4_15 = ROlp4_13*s.dpt[1,13]+s.dpt[3,13]*S19
    RLlp4_25 = ROlp4_23*s.dpt[1,13]+ROlp4_82*s.dpt[3,13]
    RLlp4_35 = ROlp4_33*s.dpt[1,13]+ROlp4_92*s.dpt[3,13]
    POlp4_15 = POlp4_14+RLlp4_15
    POlp4_25 = POlp4_24+RLlp4_25
    POlp4_35 = POlp4_34+RLlp4_35
    JTlp4_25_1 = -RLlp4_34-RLlp4_35
    JTlp4_35_1 = RLlp4_24+RLlp4_25
    JTlp4_15_2 = JTlp4_14_2-RLlp4_25*S18+RLlp4_35*C18
    JTlp4_25_2 = JTlp4_24_2+RLlp4_15*S18
    JTlp4_35_2 = JTlp4_34_2-RLlp4_15*C18
    JTlp4_15_3 = JTlp4_14_3-RLlp4_25*ROlp4_92+RLlp4_35*ROlp4_82
    JTlp4_25_3 = JTlp4_24_3+RLlp4_15*ROlp4_92-RLlp4_35*S19
    JTlp4_35_3 = JTlp4_34_3-RLlp4_15*ROlp4_82+RLlp4_25*S19
    Plp12 = -POlp4_15+RLlp3_12
    Plp22 = -POlp4_25+RLlp3_22
    Plp32 = -POlp4_35+s.dpt[3,5]
    P2lp2 = Plp12*Plp12+Plp22*Plp22+Plp32*Plp32
    l2rod2 = s.lrod[2]*s.lrod[2]
    h_2 = (0.50)*(P2lp2-l2rod2)
    Jac_2_29 = -Plp12*RLlp3_22+Plp22*RLlp3_12
    Jac_2_18 = -JTlp4_25_1*Plp22-JTlp4_35_1*Plp32
    Jac_2_19 = -JTlp4_15_2*Plp12-JTlp4_25_2*Plp22-JTlp4_35_2*Plp32
    Jac_2_20 = -JTlp4_15_3*Plp12-JTlp4_25_3*Plp22-JTlp4_35_3*Plp32
    Jac_2_21 = -Plp12*S19-Plp22*ROlp4_82-Plp32*ROlp4_92
    h[1] = h_1
    h[2] = h_2
    Jac[1,1] = 0
    Jac[1,2] = 0
    Jac[1,3] = 0
    Jac[1,4] = 0
    Jac[1,5] = 0
    Jac[1,6] = 0
    Jac[1,7] = Jac_1_7
    Jac[1,8] = Jac_1_8
    Jac[1,9] = Jac_1_9
    Jac[1,10] = Jac_1_10
    Jac[1,11] = 0
    Jac[1,12] = 0
    Jac[1,13] = 0
    Jac[1,14] = 0
    Jac[1,15] = 0
    Jac[1,16] = 0
    Jac[1,17] = 0
    Jac[1,18] = 0
    Jac[1,19] = 0
    Jac[1,20] = 0
    Jac[1,21] = 0
    Jac[1,22] = 0
    Jac[1,23] = 0
    Jac[1,24] = 0
    Jac[1,25] = 0
    Jac[1,26] = 0
    Jac[1,27] = 0
    Jac[1,28] = 0
    Jac[1,29] = Jac_1_29
    Jac[1,30] = 0
    Jac[1,31] = 0
    Jac[2,1] = 0
    Jac[2,2] = 0
    Jac[2,3] = 0
    Jac[2,4] = 0
    Jac[2,5] = 0
    Jac[2,6] = 0
    Jac[2,7] = 0
    Jac[2,8] = 0
    Jac[2,9] = 0
    Jac[2,10] = 0
    Jac[2,11] = 0
    Jac[2,12] = 0
    Jac[2,13] = 0
    Jac[2,14] = 0
    Jac[2,15] = 0
    Jac[2,16] = 0
    Jac[2,17] = 0
    Jac[2,18] = Jac_2_18
    Jac[2,19] = Jac_2_19
    Jac[2,20] = Jac_2_20
    Jac[2,21] = Jac_2_21
    Jac[2,22] = 0
    Jac[2,23] = 0
    Jac[2,24] = 0
    Jac[2,25] = 0
    Jac[2,26] = 0
    Jac[2,27] = 0
    Jac[2,28] = 0
    Jac[2,29] = Jac_2_29
    Jac[2,30] = 0
    Jac[2,31] = 0

# Number of continuation lines = 0


