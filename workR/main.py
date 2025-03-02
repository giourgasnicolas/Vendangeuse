#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""Script to run a direct dynamic analysis on a multibody system.

Summary
-------
This template loads the data file *.mbs and execute:
 - the coordinate partitioning module
 - the direct dynamic module (time integration of equations of motion).
 - if available, plot the time evolution of the first generalized coordinate.

It may have to be adapted and completed by the user.


Universite catholique de Louvain
CEREM : Centre for research in mechatronics

http://www.robotran.eu
Contact : info@robotran.be

(c) Universite catholique de Louvain
"""

# %%============================================================================
# Packages loading
# =============================================================================
from tabnanny import verbose
import numpy


try:
    import MBsysPy as Robotran
except:
    raise ImportError("MBsysPy not found/installed."
                      "See: https://www.robotran.eu/download/how-to-install/"
                      )

# %%===========================================================================
# Project loading
# =============================================================================
mbs_data = Robotran.MbsData('../dataR/Vendangeuse.mbs')
mbs_part = Robotran.MbsPart(mbs_data)
mbs_part.run()
# %%===========================================================================
# Partitionning
# =============================================================================
mbs_data.process = 1
mbs_part = Robotran.MbsPart(mbs_data)
mbs_part.set_options(rowperm=1, verbose=1)
mbs_part.run()


# # %%===========================================================================
# # EQUIL
# # =============================================================================
# mbs_data.process = 2
# mbs_equil = Robotran.MbsEquil(mbs_data)
# mbs_equil.set_options(method =1, senstol = 1e-2, verbose = 1)
# results = mbs_equil.run()
# del mbs_equil


# # %%===========================================================================
# # MODAL
# # =============================================================================
# mbs_data.process = 4
# mbs_modal = Robotran.MbsModal(mbs_data)
# mbs_modal.set_options(save_result = 1, save_anim = 1, mode_ampl = 0.2, save_eval = 1) #le dernier flag crée des fichiers .res dans resultsR
# modal_values = mbs_modal.run()

# # ###Récupération des valeurs propres
# modal_values.eval_a = numpy.loadtxt('../resultsR/modal_eval_a.res')
# modal_values.eval_b = numpy.loadtxt('../resultsR/modal_eval_b.res')
# print(modal_values.eval_a ,modal_values.eval_b)

# #calcul des fréquences
# w0 = numpy.sqrt((modal_values.eval_a)*(modal_values.eval_a) + (modal_values.eval_b)*(modal_values.eval_b)) #fréquence angulaire sans amortissement [rad/s]
# f0 = w0/(2*numpy.pi)  #[Hz]
# print("la fréquence du mouvement selon MODAL sans amortissement", f0)

# zeta = numpy.cos(numpy.arctan(-(modal_values.eval_a/modal_values.eval_b))) #[]
# w = w0*numpy.square(1-zeta*zeta) #[rad/s]
# f = w/(2*numpy.pi) #[Hz]
# print ("la fréquence du mouvement selon MODAL avec amortissement", f)
# del mbs_modal

# %%===========================================================================
# Repartitionning and reloading for DirDyn 
# =============================================================================
# mbs_data = Robotran.MbsData('../dataR/Vendangeuse.mbs')
# mbs_data.process = 1
# mbs_part = Robotran.MbsPart(mbs_data)
# mbs_part.run()

# %%===========================================================================
# Direct Dynamics
# =============================================================================
mbs_data.process = 3
mbs_dirdyn = Robotran.MbsDirdyn(mbs_data)
mbs_dirdyn.set_options(dt0=1e-3, tf= 15, save2file=1,verbose= 1)
results = mbs_dirdyn.run()


trajectoire = open("vitesse8.res", "w+")
for i in range(len(results.q[:,1])) :
    trajectoire.write(str(results.q[i,1])+'\t') #Position en x
    trajectoire.write(str(results.q[i,2])+'\t') #Position en y
    trajectoire.write(str(results.q[i,5])+'\n') # R1
    

# %%===========================================================================
# Plotting results
# =============================================================================
try:
    import matplotlib.pyplot as plt
except Exception:
    raise RuntimeError('Unable to load matplotlib, plotting results unavailable.')


#------plot de q1-------------------------

        
# Figure creation
fig = plt.figure(figsize=(8, 6))


plt.subplot(2,1,1)
plt.plot(results.q[:, 1], results.q[:, 2], label='Position du châssis')
plt.legend(markerscale = 0.5, loc = 'upper right')
plt.title('Position verticale du châssis en fonction du temps')
        


plt.subplot(2,1,2)
plt.plot(results.q[:, 1], results.q[:, 5], label='Position du châssis en x')
plt.legend(markerscale = 0.5, loc = 'upper right')
plt.title('Position en x du châssis en fonction du temps')
plt.show()

