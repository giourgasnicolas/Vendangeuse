# -*- coding: utf-8 -*-
"""
Module for the definition of the derivatives of the user states.

Summary
-------
The user derivatives allows to add some equations to be time-integrated with
the multibody system.
"""
# Author: Robotran Team
# (c) Universite catholique de Louvain, 2020

from math import sin,cos,tan
def user_derivatives(mbs_data):
    """Compute the derivatives of the user states.

    Parameters
    ----------
    mbs_data : MBsysPy.MbsData
        The instance containing the multibody project. The derivatives values
        must be filled in the 'uxd' attribute which is a numpy.ndarray. The first
        index (mbs_data.uxd[0]) must not be modified. The first index to be
        filled is mbs_data.uxd[1].

    Returns
    -------
    None
    """

    # Example: integration of sin(t) in first user state variable
    # mbs_data.uxd[1] = np.cos(mbs_data.tsim)

    ### suivant l'axe de roulis debut ###########################################
    theta = mbs_data.q[6]
    alpha = 0.1419
    L = 3.0303       #longeur du chassis   
    mbs_data.uxd[1] = ( (L*tan(theta) + mbs_data.ux[1]*cos(alpha))*tan(alpha)*( sin(theta)/cos(alpha+theta) + 1/sin(alpha))  - mbs_data.ux[1]   )
    mbs_data.uxd[2] = mbs_data.ux[2]*(1 - cos(alpha)*cos(alpha) )
    ### suivant l'axe de roulis debut ###########################################

    ### suivant l'axe de roulis debut ###########################################
    theta = mbs_data.q[5]
    alpha = 0.1146
    L = 1.12         #largeur du chassis   
    mbs_data.uxd[3] = ( (L*tan(theta) + mbs_data.ux[3]*cos(alpha))*tan(alpha)*( sin(theta)/cos(alpha+theta) + 1/sin(alpha))  - mbs_data.ux[3]   )
    mbs_data.uxd[4] = mbs_data.ux[4]*(1 - cos(alpha)*cos(alpha) )
    ### suivant l'axe de roulis debut ###########################################
    return
