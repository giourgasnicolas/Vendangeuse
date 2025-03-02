# -*- coding: utf-8 -*-
"""Module for the definition of driven joints."""
# Author: Robotran Team

import numpy as np
from MBsysPy import MbsSensor
from math import sin,cos
def user_DrivenJoints(mbs_data, tsim):
    """Set the values of the driven joints directly in the MbsData structure.

    The position, velocity and acceleration of the driven joints must be set in
    the attributes mbs_data.q, mbs_data.qd and mbs_data.qdd .

    Parameters
    ----------
    mbs_data : MBsysPy.MbsData
        The multibody system associated to this computation.
    tsim : float
        The current time of the simulation.

    Returns
    -------
    None
    """

    Evite = True
    
    MiseNiveau = False
    
    #---Rotation des roues arrières----
    
    id_j = mbs_data.joint_id["Joint_1"]
    omega = 8
    #pour tester le sloshing en ligne droite
    # if tsim >= 3 :
    #      omega*=5
    # if tsim >= 6 :
    #      omega/=25 
    mbs_data.q[id_j]   = omega*tsim
    mbs_data.qd[id_j]  = omega
    mbs_data.qdd[id_j] = 0

    id_j = mbs_data.joint_id["Joint_4"]
    omega = 8
    #pour tester le sloshing en ligne droite
    # if tsim >= 3 :
    #     omega*=5
    # if tsim >= 6 :
    #     omega/=25
    mbs_data.q[id_j]   = omega*tsim
    mbs_data.qd[id_j]  = omega
    mbs_data.qdd[id_j] = 0
        
  
    if MiseNiveau == True :
        ### mise a niveau debut #############################################################################
    
        ### suivant l'axe de tanguage debut ###########################################
        theta = mbs_data.q[6]
        alpha = 0.1419
        L = 3.0303       #longeur du chassis
        l = 2.0          #longeur du verrin
    
        #---verrin arriere gauche----
        id_j = mbs_data.joint_id["Joint_19"]
        mbs_data.q[id_j]   = mbs_data.ux[1]
        mbs_data.qd[id_j]  = mbs_data.uxd[1]
        mbs_data.qdd[id_j] = (L + mbs_data.ux[1]*sin(alpha))*cos(alpha)/cos(theta + alpha)**2
    
        #---verrin arriere droit----
        id_j = mbs_data.joint_id["Joint_20"]
        mbs_data.q[id_j]   = mbs_data.ux[1]
        mbs_data.qd[id_j]  = mbs_data.uxd[1]
        mbs_data.qdd[id_j] = (L + mbs_data.ux[1]*sin(alpha))*cos(alpha)/cos(theta + alpha)**2
    
        #---verrin avant gauche----  
        id_j = mbs_data.joint_id["Joint_18"]
        mbs_data.q[id_j]   = mbs_data.ux[2]
        mbs_data.qd[id_j]  = mbs_data.uxd[2]
        mbs_data.qdd[id_j] = (1 - cos(alpha)*cos(alpha) )
    
        #---verrin avant droit----
        id_j = mbs_data.joint_id["Joint_21"]
        mbs_data.q[id_j]   = mbs_data.ux[2]
        mbs_data.qd[id_j]  = mbs_data.uxd[2]
        mbs_data.qdd[id_j] = (1 - cos(alpha)*cos(alpha) )  
        ### suivant l'axe de tanguage fin #############################################
    
        # ### suivant l'axe de roulis debut ###########################################
        # theta = mbs_data.q[6]
        # alpha = 0.1419
        # L = 3.0303       #longeur du chassis
        # l = 2.0          #longeur du verrin
    
        # #---verrin arriere gauche----
        # id_j = mbs_data.joint_id["Joint_19"]
        # mbs_data.q[id_j]   = mbs_data.ux[4]
        # mbs_data.qd[id_j]  = mbs_data.uxd[4]
        # mbs_data.qdd[id_j] = (1 - cos(alpha)*cos(alpha) )
    
        # #---verrin arriere droit----
        # id_j = mbs_data.joint_id["Joint_20"]
        # mbs_data.q[id_j]   = mbs_data.ux[3]
        # mbs_data.qd[id_j]  = mbs_data.uxd[3]
        # mbs_data.qdd[id_j] = (L + mbs_data.ux[3]*sin(alpha))*cos(alpha)/cos(theta + alpha)**2
    
        # #---verrin avant gauche----  
        # id_j = mbs_data.joint_id["Joint_18"]
        # mbs_data.q[id_j]   = mbs_data.ux[4]
        # mbs_data.qd[id_j]  = mbs_data.uxd[4]
        # mbs_data.qdd[id_j] = (1 - cos(alpha)*cos(alpha) )
        # #---verrin avant droit----
        # id_j = mbs_data.joint_id["Joint_21"]
        # mbs_data.q[id_j]   = mbs_data.ux[3]
        # mbs_data.qd[id_j]  = mbs_data.uxd[3]
        # mbs_data.qdd[id_j] = (L + mbs_data.ux[3]*sin(alpha))*cos(alpha)/cos(theta + alpha)**2
    
        # ### suivant l'axe de roulis fin #############################################
  
    ### mise a niveau fin ###############################################################################
#---Evitement--------------------
    
    
    if Evite == True :
        
        evite(mbs_data,tsim, 10, 30, 3)
            

    return

def evite(mbs_data,tsim, xi, d, A):
    """
    Effectue une manoeuvre d'évitement sinisoidale en fonction des paramètres données

    Parametres
    ----------
    mbs_data : MBsysPy.MbsData
        Le système multicorps associé à la résolution.
        
    tsim : float
        Le temps actuelle de la simulation.
        

    Retourne
    -------
    Rien
    """    
    
    id_direction = mbs_data.joint_id["Joint_0_dir"]
    
    K = 5**(-1)
    
    f = 1/(2*d)
    
    sensor = MbsSensor(mbs_data)

    id_sensor_1 = 1

    sensor.comp_s_sensor(id_sensor_1)
    
    y_route = 0.0
    x       = sensor.P[1]
    y       = sensor.P[2]
    
    if (abs(y)<1e-14) : 
        y = 0.0        #Parce qu'il y a des petit glissement 

    
    if x >= xi and x <= xi + d :
    
        y_route = A*np.sin(2*np.pi*(x- xi)*f)
    
    else :
        
        y_route = 0.0
    
    err = y - y_route
    
    mbs_data.q[id_direction] = -K*err
    
    return
