import numpy as np
from matplotlib import pyplot as plt 

  
m_chassis = 4000
a = 3.35
b = 3
c = 2.5
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

I_chassis = inertie_parallélépipede_plein(m_chassis, a, b, c)

I_tige = inertie_cylindre_plein(m_tige, R_tige, H_tige)

I_roue = inertie_cylindre_plein(m_roue, R_roue, H_roue)

I_roue2 = inertie_cylindre_plein(m_roue2, R_roue2, H_roue2)

print("Matrice d'inertie du chassis :"+"\n"+ str(I_chassis)+"\n")
print("Matrice d'inertie de la tige :"+"\n"+ str(I_tige)+"\n")
print("Matrice d'inertie de la roue avant :"+"\n"+ str(I_roue)+"\n")
print("Matrice d'inertie de la roue arrière :"+"\n"+ str(I_roue2)+"\n")
