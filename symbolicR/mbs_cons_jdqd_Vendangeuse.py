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
#	==> Function: F18 - Constraints Quadratic Velocity Terms (Jdqd)
#
#	==> Git hash: 220d76a94da8547597795a13c9921f0375de3fc1
#

from math import sin, cos

def cons_jdqd(Jdqd, s):
    q = s.q
    qd = s.qd
 
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

 
# Constraints Quadratic Terms

    RLjdqd1_12 = s.dpt[1,18]*C29-s.dpt[2,18]*S29
    RLjdqd1_22 = s.dpt[1,18]*S29+s.dpt[2,18]*C29
    ORjdqd1_12 = -RLjdqd1_22*qd[29]
    ORjdqd1_22 = RLjdqd1_12*qd[29]
    Apqpjdqd1_12 = -ORjdqd1_22*qd[29]
    Apqpjdqd1_22 = ORjdqd1_12*qd[29]
    ROjdqd2_22 = S7*S8
    ROjdqd2_32 = -C7*S8
    ROjdqd2_82 = -S7*C8
    ROjdqd2_92 = C7*C8
    ROjdqd2_13 = C8*C9
    ROjdqd2_23 = ROjdqd2_22*C9+C7*S9
    ROjdqd2_33 = ROjdqd2_32*C9+S7*S9
    OMjdqd2_22 = qd[8]*C7
    OMjdqd2_32 = qd[8]*S7
    Ompqpjdqd2_22 = -qd[7]*qd[8]*S7
    Ompqpjdqd2_32 = qd[7]*qd[8]*C7
    OMjdqd2_13 = qd[7]+qd[9]*S8
    OMjdqd2_23 = OMjdqd2_22+ROjdqd2_82*qd[9]
    OMjdqd2_33 = OMjdqd2_32+ROjdqd2_92*qd[9]
    Ompqpjdqd2_13 = qd[9]*(OMjdqd2_22*ROjdqd2_92-OMjdqd2_32*ROjdqd2_82)
    Ompqpjdqd2_23 = Ompqpjdqd2_22+qd[9]*(OMjdqd2_32*S8-ROjdqd2_92*qd[7])
    Ompqpjdqd2_33 = Ompqpjdqd2_32+qd[9]*(-OMjdqd2_22*S8+ROjdqd2_82*qd[7])
    RLjdqd2_14 = q[10]*S8
    RLjdqd2_24 = ROjdqd2_82*q[10]
    RLjdqd2_34 = ROjdqd2_92*q[10]
    POjdqd2_14 = RLjdqd2_14+s.dpt[1,1]
    POjdqd2_24 = RLjdqd2_24+s.dpt[2,1]
    POjdqd2_34 = RLjdqd2_34+s.dpt[3,1]
    ORjdqd2_14 = OMjdqd2_23*RLjdqd2_34-OMjdqd2_33*RLjdqd2_24
    ORjdqd2_24 = -OMjdqd2_13*RLjdqd2_34+OMjdqd2_33*RLjdqd2_14
    ORjdqd2_34 = OMjdqd2_13*RLjdqd2_24-OMjdqd2_23*RLjdqd2_14
    VIjdqd2_14 = ORjdqd2_14+qd[10]*S8
    VIjdqd2_24 = ORjdqd2_24+ROjdqd2_82*qd[10]
    VIjdqd2_34 = ORjdqd2_34+ROjdqd2_92*qd[10]
    Apqpjdqd2_14 = OMjdqd2_23*ORjdqd2_34-OMjdqd2_33*ORjdqd2_24+Ompqpjdqd2_23*RLjdqd2_34-Ompqpjdqd2_33*RLjdqd2_24+(2.0)*qd[10]*(OMjdqd2_23*ROjdqd2_92-OMjdqd2_33*ROjdqd2_82)
    Apqpjdqd2_24 = -OMjdqd2_13*ORjdqd2_34+OMjdqd2_33*ORjdqd2_14-Ompqpjdqd2_13*RLjdqd2_34+Ompqpjdqd2_33*RLjdqd2_14+(2.0)*qd[10]*(-OMjdqd2_13*ROjdqd2_92+OMjdqd2_33*S8)
    Apqpjdqd2_34 = OMjdqd2_13*ORjdqd2_24-OMjdqd2_23*ORjdqd2_14+Ompqpjdqd2_13*RLjdqd2_24-Ompqpjdqd2_23*RLjdqd2_14+(2.0)*qd[10]*(OMjdqd2_13*ROjdqd2_82-OMjdqd2_23*S8)
    RLjdqd2_15 = ROjdqd2_13*s.dpt[1,8]+s.dpt[3,8]*S8
    RLjdqd2_25 = ROjdqd2_23*s.dpt[1,8]+ROjdqd2_82*s.dpt[3,8]
    RLjdqd2_35 = ROjdqd2_33*s.dpt[1,8]+ROjdqd2_92*s.dpt[3,8]
    POjdqd2_15 = POjdqd2_14+RLjdqd2_15
    POjdqd2_25 = POjdqd2_24+RLjdqd2_25
    POjdqd2_35 = POjdqd2_34+RLjdqd2_35
    ORjdqd2_15 = OMjdqd2_23*RLjdqd2_35-OMjdqd2_33*RLjdqd2_25
    ORjdqd2_25 = -OMjdqd2_13*RLjdqd2_35+OMjdqd2_33*RLjdqd2_15
    ORjdqd2_35 = OMjdqd2_13*RLjdqd2_25-OMjdqd2_23*RLjdqd2_15
    VIjdqd2_15 = ORjdqd2_15+VIjdqd2_14
    VIjdqd2_25 = ORjdqd2_25+VIjdqd2_24
    VIjdqd2_35 = ORjdqd2_35+VIjdqd2_34
    Apqpjdqd2_15 = Apqpjdqd2_14+OMjdqd2_23*ORjdqd2_35-OMjdqd2_33*ORjdqd2_25+Ompqpjdqd2_23*RLjdqd2_35-Ompqpjdqd2_33*RLjdqd2_25
    Apqpjdqd2_25 = Apqpjdqd2_24-OMjdqd2_13*ORjdqd2_35+OMjdqd2_33*ORjdqd2_15-Ompqpjdqd2_13*RLjdqd2_35+Ompqpjdqd2_33*RLjdqd2_15
    Apqpjdqd2_35 = Apqpjdqd2_34+OMjdqd2_13*ORjdqd2_25-OMjdqd2_23*ORjdqd2_15+Ompqpjdqd2_13*RLjdqd2_25-Ompqpjdqd2_23*RLjdqd2_15
    jdqd1 = -Apqpjdqd2_35*(-POjdqd2_35+s.dpt[3,5])+VIjdqd2_35*VIjdqd2_35+(-POjdqd2_15+RLjdqd1_12)*(Apqpjdqd1_12-Apqpjdqd2_15)+(-POjdqd2_25+RLjdqd1_22)*(Apqpjdqd1_22-Apqpjdqd2_25)+(ORjdqd1_12-VIjdqd2_15)*(ORjdqd1_12-VIjdqd2_15)+(ORjdqd1_22-VIjdqd2_25)*(ORjdqd1_22-VIjdqd2_25)
    RLjdqd3_12 = s.dpt[1,17]*C29-s.dpt[2,17]*S29
    RLjdqd3_22 = s.dpt[1,17]*S29+s.dpt[2,17]*C29
    ORjdqd3_12 = -RLjdqd3_22*qd[29]
    ORjdqd3_22 = RLjdqd3_12*qd[29]
    Apqpjdqd3_12 = -ORjdqd3_22*qd[29]
    Apqpjdqd3_22 = ORjdqd3_12*qd[29]
    ROjdqd4_22 = S18*S19
    ROjdqd4_32 = -C18*S19
    ROjdqd4_82 = -S18*C19
    ROjdqd4_92 = C18*C19
    ROjdqd4_13 = C19*C20
    ROjdqd4_23 = ROjdqd4_22*C20+C18*S20
    ROjdqd4_33 = ROjdqd4_32*C20+S18*S20
    OMjdqd4_22 = qd[19]*C18
    OMjdqd4_32 = qd[19]*S18
    Ompqpjdqd4_22 = -qd[18]*qd[19]*S18
    Ompqpjdqd4_32 = qd[18]*qd[19]*C18
    OMjdqd4_13 = qd[18]+qd[20]*S19
    OMjdqd4_23 = OMjdqd4_22+ROjdqd4_82*qd[20]
    OMjdqd4_33 = OMjdqd4_32+ROjdqd4_92*qd[20]
    Ompqpjdqd4_13 = qd[20]*(OMjdqd4_22*ROjdqd4_92-OMjdqd4_32*ROjdqd4_82)
    Ompqpjdqd4_23 = Ompqpjdqd4_22+qd[20]*(OMjdqd4_32*S19-ROjdqd4_92*qd[18])
    Ompqpjdqd4_33 = Ompqpjdqd4_32+qd[20]*(-OMjdqd4_22*S19+ROjdqd4_82*qd[18])
    RLjdqd4_14 = q[21]*S19
    RLjdqd4_24 = ROjdqd4_82*q[21]
    RLjdqd4_34 = ROjdqd4_92*q[21]
    POjdqd4_14 = RLjdqd4_14+s.dpt[1,3]
    POjdqd4_24 = RLjdqd4_24+s.dpt[2,3]
    POjdqd4_34 = RLjdqd4_34+s.dpt[3,3]
    ORjdqd4_14 = OMjdqd4_23*RLjdqd4_34-OMjdqd4_33*RLjdqd4_24
    ORjdqd4_24 = -OMjdqd4_13*RLjdqd4_34+OMjdqd4_33*RLjdqd4_14
    ORjdqd4_34 = OMjdqd4_13*RLjdqd4_24-OMjdqd4_23*RLjdqd4_14
    VIjdqd4_14 = ORjdqd4_14+qd[21]*S19
    VIjdqd4_24 = ORjdqd4_24+ROjdqd4_82*qd[21]
    VIjdqd4_34 = ORjdqd4_34+ROjdqd4_92*qd[21]
    Apqpjdqd4_14 = OMjdqd4_23*ORjdqd4_34-OMjdqd4_33*ORjdqd4_24+Ompqpjdqd4_23*RLjdqd4_34-Ompqpjdqd4_33*RLjdqd4_24+(2.0)*qd[21]*(OMjdqd4_23*ROjdqd4_92-OMjdqd4_33*ROjdqd4_82)
    Apqpjdqd4_24 = -OMjdqd4_13*ORjdqd4_34+OMjdqd4_33*ORjdqd4_14-Ompqpjdqd4_13*RLjdqd4_34+Ompqpjdqd4_33*RLjdqd4_14+(2.0)*qd[21]*(-OMjdqd4_13*ROjdqd4_92+OMjdqd4_33*S19)
    Apqpjdqd4_34 = OMjdqd4_13*ORjdqd4_24-OMjdqd4_23*ORjdqd4_14+Ompqpjdqd4_13*RLjdqd4_24-Ompqpjdqd4_23*RLjdqd4_14+(2.0)*qd[21]*(OMjdqd4_13*ROjdqd4_82-OMjdqd4_23*S19)
    RLjdqd4_15 = ROjdqd4_13*s.dpt[1,13]+s.dpt[3,13]*S19
    RLjdqd4_25 = ROjdqd4_23*s.dpt[1,13]+ROjdqd4_82*s.dpt[3,13]
    RLjdqd4_35 = ROjdqd4_33*s.dpt[1,13]+ROjdqd4_92*s.dpt[3,13]
    POjdqd4_15 = POjdqd4_14+RLjdqd4_15
    POjdqd4_25 = POjdqd4_24+RLjdqd4_25
    POjdqd4_35 = POjdqd4_34+RLjdqd4_35
    ORjdqd4_15 = OMjdqd4_23*RLjdqd4_35-OMjdqd4_33*RLjdqd4_25
    ORjdqd4_25 = -OMjdqd4_13*RLjdqd4_35+OMjdqd4_33*RLjdqd4_15
    ORjdqd4_35 = OMjdqd4_13*RLjdqd4_25-OMjdqd4_23*RLjdqd4_15
    VIjdqd4_15 = ORjdqd4_15+VIjdqd4_14
    VIjdqd4_25 = ORjdqd4_25+VIjdqd4_24
    VIjdqd4_35 = ORjdqd4_35+VIjdqd4_34
    Apqpjdqd4_15 = Apqpjdqd4_14+OMjdqd4_23*ORjdqd4_35-OMjdqd4_33*ORjdqd4_25+Ompqpjdqd4_23*RLjdqd4_35-Ompqpjdqd4_33*RLjdqd4_25
    Apqpjdqd4_25 = Apqpjdqd4_24-OMjdqd4_13*ORjdqd4_35+OMjdqd4_33*ORjdqd4_15-Ompqpjdqd4_13*RLjdqd4_35+Ompqpjdqd4_33*RLjdqd4_15
    Apqpjdqd4_35 = Apqpjdqd4_34+OMjdqd4_13*ORjdqd4_25-OMjdqd4_23*ORjdqd4_15+Ompqpjdqd4_13*RLjdqd4_25-Ompqpjdqd4_23*RLjdqd4_15
    jdqd2 = -Apqpjdqd4_35*(-POjdqd4_35+s.dpt[3,5])+VIjdqd4_35*VIjdqd4_35+(-POjdqd4_15+RLjdqd3_12)*(Apqpjdqd3_12-Apqpjdqd4_15)+(-POjdqd4_25+RLjdqd3_22)*(Apqpjdqd3_22-Apqpjdqd4_25)+(ORjdqd3_12-VIjdqd4_15)*(ORjdqd3_12-VIjdqd4_15)+(ORjdqd3_22-VIjdqd4_25)*(ORjdqd3_22-VIjdqd4_25)
    Jdqd[1] = jdqd1
    Jdqd[2] = jdqd2

# Number of continuation lines = 0


