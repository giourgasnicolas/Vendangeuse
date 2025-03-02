# -*- coding: utf-8 -*-
"""Module for the definition of joint forces."""
# Author: Robotran Team
# (c) Universite catholique de Louvain, 2020


from numpy import sign


def user_JointForces(mbs_data, tsim):
    """Compute the force and torques in the joint.

    It fills the MBsysPy.MbsData.Qq array.

    Parameters
    ----------
    mbs_data : MBsysPy.MbsData
        The multibody system associated to this computation.
    tsim : float
        The current time of the simulation.

    Notes
    -----
    The numpy.ndarray MBsysPy.MbsData.Qq is 1D array with index starting at 1.
    The first index (array[0]) must not be modified. The first index to be
    filled is array[1].

    Returns
    -------
    None
    """
    D = 10000
    K = 150000
    # L0 = 2
    #vendangeuse_amo
    for i in [10,16,23,29]: #les indices des translations dnas les tiges
        mbs_data.Qq[i] = -D * mbs_data.qd[i] - K*(mbs_data.q[i])

    # D = 10e8
    # K = 76e2

    #masse du réservoir rempli à moitié : 1694.925 kg
    
    # if mbs_data.q[34] <1.5 or mbs_data.q[34]>-1.5:
    #     mbs_data.Qq[34] = -(D * mbs_data.qd[34] + K*(mbs_data.q[34])) #sign(mbs_data.q[30])*
        
    # if mbs_data.q[35] <1.675 or mbs_data.q[35]>-1.675:
    #     mbs_data.Qq[35] = -(D * mbs_data.qd[35] + K*(mbs_data.q[35]))

    return
