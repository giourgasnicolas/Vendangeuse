import numpy as np
from matplotlib import pyplot as plt
from pyrsistent import b 

  
m_chassis = 4000
a_c = 3.35
b_c = 3
c_c = 2.5
#--------tige---------------
m_tige = 100  
H_tige = 2
R_tige = 0.1
#--------roue_avant---------------
H_roue = 0.42
m_roue = 200
R_roue = 0.6
#--------roue_arrière---------------
H_roue2 = 0.48
m_roue2 = 300
R_roue2 = 0.7
#--------bloc---------------
m_bloc = 5
a_bloc = 0.22
b_bloc = 0.14
c_bloc = 0.14
#--------barre---------------
m_barre = 15
H_barre = 0.72
R_barre = 0.07   
#--------cone_dir------
H_cone = 0.320
m_cone = 10 
R_cone = 0.150  
#--------Réservoir---------------
a_res = 1.674
b_res = 3
c_res = 0.675
m_res = 1000* a_res*b_res*c_res/2  #rempli à moitié on néglige sa masse vide


def inertie_cylindre_plein(m, R, H) :
    I1 = m*R*R/4 + m*H*H/12
    I2 = I1
    I3 = m*R*R/2
    return np.array([[I1, 0, 0],[0, I2, 0],[0, 0, I3]])

def inertie_parallélépipede_plein(m, a, b, c) :
    I1 = m*(b*b + c*c)/12
    I2 = m*(a*a + c*c)/12
    I3 = m*(a*a + b*b)/12
    return np.array([[I1, 0, 0],[0, I2, 0],[0, 0, I3]])

def inertie_cone_plein(m, R, H) :
    const = 3*m/10
    I1 = const*(2*H*H + (R*R)/2)
    I2 = I1
    I3 = const*R*R
    return np.array([[I1, 0, 0],[0, I2, 0],[0, 0, I3]])

# def axes_paral(forme, aire, dist, caract) :
#     """ 
#     pre : 
#         forme : str de la géométrie : cylindre ou parallélépipède
#         aire : list des aires des surfaces associèes (je sais pas comment mieux les définir)
#         dist : list des  distances entre l'origine et le centre de gravité dans les directions X Y et Z 
#                 la distance pour X est le déplacement dans toutes les directions sauf X (les déplacement sur X ne changeront pas Ix)
#         caract : list des longueurs caractéristiques de la forme et de la masse (en 0)
    
#     post : 
#         la matrice d'inertie
#     """

#     if forme == "parallélépipède":
#         m,a,b,c = caract
#         matI = inertie_parallélépipede_plein(m, a, b, c)

#     elif forme == "cylindre":
#         m,R,H = caract
#         matI = inertie_cylindre_plein(m, R, H)
    
#     else : print("youstidiou c'est pas une forme qu'on prend en compte ça :-)")

#     for i in range(3) : 
#         matI[i][i] += aire[i]*dist[i]*dist[i]
    
#     return matI

#--------------Sans changement de base-------------------------------------------------------------------

I_chassis = inertie_parallélépipede_plein(m_chassis, a_c, b_c, c_c)

I_tige = inertie_cylindre_plein(m_tige, R_tige, H_tige)

I_roue = inertie_cylindre_plein(m_roue, R_roue, H_roue)

I_roue2 = inertie_cylindre_plein(m_roue2, R_roue2, H_roue2)

I_barre = inertie_cylindre_plein(m_barre, R_barre, H_barre) 

I_bloc = inertie_parallélépipede_plein(m_bloc, a_bloc, b_bloc, c_bloc)

I_cone = inertie_cone_plein(m_cone, R_cone, H_cone)

I_res = inertie_parallélépipede_plein(m_res, a_res, b_res, c_res)

print("Matrice d'inertie du chassis :"+"\n"+ str(I_chassis)+"\n")
print("Matrice d'inertie de la tige :"+"\n"+ str(I_tige)+"\n")
print("Matrice d'inertie de la roue avant :"+"\n"+ str(I_roue)+"\n")
print("Matrice d'inertie de la roue arrière :"+"\n"+ str(I_roue2)+"\n")

print("Matrice d'inertie du bloc :"+"\n"+ str(I_bloc)+"\n")
print("Matrice d'inertie de la barre :"+"\n"+ str(I_barre)+"\n")
print("Matrice d'inertie du cone :"+"\n"+ str(I_cone)+"\n")

print("Matrice d'inertie du réservoir :"+"\n"+ str(I_res)+"\n")
print("masse réservoir : ",m_res)