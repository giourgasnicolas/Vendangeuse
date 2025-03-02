# -*- coding: utf-8 -*-
"""Module for defining user function required to compute external forces."""
# Author: Robotran Team
# (c) Universite catholique de Louvain, 2019
import numpy as np
from mbs_tgc import *
from MBsysPy.mbsyspy.utilities.algebra import matrix_matrix_product, matrix_vector_product, cross_product



def bosse_fct(PxF, mbs_data, tsim, start, pen, A = 0.2, long = 5,forme = "rampe") :
    """
    Effectue ule passage sur une bosse ayant la forme d'un sinus.
    
    Parametre
    ----------
    PxF : numpy.ndarray
        Position vector (index starting at 1) of the force sensor expressed in
        the inertial frame: PxF[1:4] = [P_x, P_y, P_z]

    mbs_data : MBsysPy.MbsData
        Le système multicorps associé à la résolution
        
    tsim : float
        Temps actuel de la simulation
    
    start : float
        Position en x d'apparition de la bosse 
    
    pen : float 
        Déformation du pneu 
    
    A : int 
        la hauteur max de la bosse
    
    long : float
        la distance entre le début et la fin de la bosse ( au plus long au plus doux)

    forme : string
        la forme de la bosse :"rampe", "sinus" ou "triangle" dans l'ordre du pire au meilleur 
        
    Retourne
    -------
    Rien
    """ 
    bosse = 0
    end = start + long #Pour que la bosse s'arrête après une amplitude de sinus 
    
    if PxF[1]>= start and PxF[1]<= end :
        if forme == "sinus" :
            bosse = A*np.sin(np.pi*(PxF[1]-start)/long)
        
        if forme == "rampe" :
            if PxF[1]<= start + long/3 :
                bosse = A*(PxF[1]-start)/(long/3)
            if PxF[1] > start + long/3 and PxF[1] < start + 2*long/3 :
                bosse = A
            if PxF[1]>= start + 2*long/3 :
                bosse = A*(end-PxF[1])/(long/3)

        if forme == "triangle" :
            if PxF[1]<= start + long/2 :
                bosse = A*(PxF[1]-start)/(long/2)
            if PxF[1]> start + long/2 :
                bosse = A*(end-PxF[1])/(long/2)

        #le pire
        # if forme == "cosinus" :
        #     bosse = -A/2*np.cos(2*np.pi*(PxF[1]-start)/long) + A/2

    return bosse

def user_ExtForces(PxF, RxF, VxF, OMxF, AxF, OMPxF, mbs_data, tsim, ixF):
    """Compute an user-specified external force.

    Parameters
    ----------
    PxF : numpy.ndarray
        Position vector (index starting at 1) of the force sensor expressed in
        the inertial frame: PxF[1:4] = [P_x, P_y, P_z]
    RxF : numpy.ndarray
        Rotation matrix (index starting at 1) from the inertial frame to the
        force sensor frame: Frame_sensor = RxF[1:4,1:4] * Frame_inertial
    VxF : numpy.ndarray
        Velocity vector (index starting at 1) of the force sensor expressed in
        the inertial frame: VxF[1:4] = [V_x, V_y, V_z]
    OMxF : numpy.ndarray
        Angular velocity vector (index starting at 1) of the force sensor
        expressed in the inertial frame: OMxF[1:4] = [OM_x, OM_y, OM_z]
    AxF : numpy.ndarray
        Acceleration vector (index starting at 1) of the force sensor expressed
        in the inertial frame: AxF[1:4] = [A_x, A_y, A_z]
    OMPxF : numpy.ndarray
        Angular acceleration vector (index starting at 1) of the force sensor
        expressed in the inertial frame: OMPxF[1:4] = [OMP_x, OMP_y, OMP_z]
    mbs_data : MBsysPy.MbsData
        The multibody system associated to this computation.
    tsim : float
        The current time of the simulation.
    ixF : int
        The ID identifying the computed force sensor.

    Notes
    -----
    For 1D numpy.ndarray with index starting at 1, the first index (array[0])
    must not be modified. The first index to be filled is array[1].

    For 2D numpy.ndarray with index starting at 1, the first row (mat[0, :]) and
    line (mat[:,0]) must not be modified. The subarray to be filled is mat[1:, 1:].

    Returns
    -------
    Swr : numpy.ndarray
        An array of length 10 equal to [0., Fx, Fy, Fz, Mx, My, Mz, dxF].
        F_# are the forces components expressed in inertial frame.
        M_# are the torques components expressed in inertial frame.
        dxF is an array of length 3 containing the component of the forces/torque
        application point expressed in the BODY-FIXED frame.
        This array is a specific line of MbsData.SWr.
    """

    Fx = 0.0
    Fy = 0.0
    Fz = 0.0
    Mx = 0.0
    My = 0.0
    Mz = 0.0
    idpt = mbs_data.xfidpt[ixF]
    dxF = mbs_data.dpt[1:, idpt]
    
    Bosse  = False 

    F1_id = mbs_data.extforce_id["Fext_Av_D"]
    F2_id = mbs_data.extforce_id["Fext_Ar_D"]
    F3_id = mbs_data.extforce_id["Fext_Ar_G"]
    F4_id = mbs_data.extforce_id["Fext_Av_G"]
    Kp = 200000      #[N/m]
    Dp = 150         #[Ns/m]

    #il faudra changer les R0 en fonction de l'avant et l'arrière
      
    if ixF == F1_id or ixF == F4_id :
        R0 = 0.6 #[m]
      
    else : 
        R0 = 0.7 #[m]

    pen, rz, angslip, angcamb, slip, Pct, Vmct, Rt_ground, f_app = tgc_car_kine_wheel(PxF, RxF, VxF, OMxF, R0)
    ed = Vmct[3]
        
    if mbs_data.process == 3 : 
        if ixF == F1_id or ixF == F2_id or ixF == F3_id or ixF == F4_id:
            if Bosse == True :
                # print("waoo1")
                start = 1 #[m]
                A = 0.2    #[m]
                long = 4   #[m]
                up = bosse_fct(PxF, mbs_data, tsim, start, pen, A, long, "sinus" )
                pen += up
            # pen += 0.08*PxF[2] #à retirer pour equil

            if pen < 0 : 
                Fz = 0
            else :
                Fz = Kp*pen + Dp*ed

            dxF[0] = f_app[1]
            dxF[1] = f_app[2]
            dxF[2] = f_app[3]


            Fwhl = matrix_vector_product(Rt_ground, np.array([0, Fx, Fy, Fz])) #petit doute sur la transposée

            Mwhl = matrix_vector_product(Rt_ground, cross_product(Pct, [0, Fx, Fy, Fz]))

            tgc_bakker_contact(Fwhl, Mwhl, angslip, angcamb, slip)

            F = matrix_vector_product(Rt_ground.T, Fwhl )
            M = matrix_vector_product(Rt_ground.T, Mwhl )


            Fx = F[1]
            Fy = F[2]
            Fz = F[3]
            Mx = M[1]
            My = M[2]
            Mz = M[3]

    #Equil
    else : 
        if ixF == F1_id or ixF == F2_id or ixF == F3_id or ixF == F4_id:
            Fz = Kp*pen + Dp*ed

    # print(dxF)
    Swr = mbs_data.SWr[ixF]
    Swr[1:] = [Fx, Fy, Fz, Mx, My, Mz, dxF[0], dxF[1], dxF[2]]
    return Swr
