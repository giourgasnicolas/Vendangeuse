# -*- coding: utf-8 -*-
"""Module for the definition of functions related to Equilibrium analysis."""
# Author: Robotran Team
# (c) Universite catholique de Louvain, 2020
from MBsysPy import MbsSensor


def user_dirdyn_init(mbs_data, mbs_dirdyn):
    """Run specific operation required by the user before running direct dynamic.

    Parameters
    ----------
    mbs_data : MBsysPy.MbsData
        The instance containing the multibody project.
    mbs_dirdyn : MBsysPy.MbsDirdyn
        The instance of the current direct dynamic process.

    Returns
    -------
    None.

    """

    # Example: Creating and storing a sensor in mbs_data then create a list to store
    #          the vertical velocity of the sensor. Two new fields are added to
    #          the MbsData instance to store the sensor and the list.
    #
    # import MBsysPy as Robotran # Should be done outside the function.
    #
    # mbs_data.my_sensor = Robotran.MbsSensor(mbs_data)
    # mbs_data.my_sensor_v = []

    return


def user_dirdyn_loop(mbs_data, mbs_dirdyn):
    """Run specific operation required by the user at the end of each integrator step.

    In case of multistep integrator, this function is not called at intermediate
    steps.

    Parameters
    ----------
    mbs_data : MBsysPy.MbsData
        The instance containing the multibody project.
    mbs_dirdyn : MBsysPy.MbsDirdyn
        The instance of the current direct dynamic process.

    Returns
    -------
    None.
    """

    # Example: The sensor, created in user_dirdyn_init, is computed (fields
    #          are updated) and the vertical velocity is added in the list.
    #
   # Creating the sensors
    my_sensor_1 = MbsSensor(mbs_data)

    # Defining the sensor ids as 1 and 4 (assuming your project has 4 sensors).
    id_sensor_1 = 1


    # Computing the sensors
    my_sensor_1.comp_s_sensor(id_sensor_1)

    # Add different outputs:
    mbs_data.set_output(my_sensor_1.P[2], "sensor_yposition_avant")
    mbs_data.set_output(my_sensor_1.P[1], "sensor_xposition_avant") 

    return


def user_dirdyn_finish(mbs_data, mbs_dirdyn):
    """Run specific operations required by the user when direct dynamic analysis ends.

    Parameters
    ----------
    mbs_data : MBsysPy.MbsData
        The instance containing the multibody project.
    mbs_dirdyn : MBsysPy.MbsDirdyn
        The instance of the current direct dynamic process.

    Returns
    -------
    None.

    """

    # Example: The velocities are saved to a file, then the fields created in
    #          the MbsData instance during the function user_dirdyn_init are
    #          removed.
    #
    # import numpy as np # Should be done outside the function.
    #
    # np.savetxt("myfile.txt", mbs_data.my_sensor_v)
    # del mbs_data.my_sensor, mbs_data.my_sensor_v

    return