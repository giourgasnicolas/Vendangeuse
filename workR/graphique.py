from matplotlib import rcParams
new_rc_params = {'text.usetex' : False, "svg.fonttype" : 'none'}
rcParams.update(new_rc_params)
import matplotlib.pyplot as plt 
import numpy as np 


traj = np.loadtxt('trajectoire.res')
traj5 = np.loadtxt('trajectoire5.res')
pente5 = np.loadtxt('pente0.05.res')
pente8 = np.loadtxt('pente0.08.res')

renv8 = np.loadtxt('renversement0.08.res')

#-------------Algorithme Evitement---------------------(xi = 10, d = 30, A = 2, v = 20km/h)

#Pour une vitesse de 20 km/h

y_route = np.zeros(len(traj[:,0]))

for i in range(len(traj[:,0])) : 
                        
    if traj[i,0] >= 10 and traj[i,0] <= 40 :
    
        y_route[i] = 2*np.sin(2*np.pi*(traj[i,0] - 10)/60)

                        
# figure1 = plt.figure("Algo", figsize=(12, 3))

# plt.plot(traj[:,0], traj[:,1], label = "Trajectoire suivie par le véhicule")
# plt.plot(traj[:,0], y_route, '--', label = "Trajectoire imposée")
# plt.xlabel("x [m]")
# plt.ylabel("y [m]")
# plt.legend(loc = 'upper right')
# plt.title("Modèle de pilotage")

# figure1.savefig("plot/pilote.png")
# plt.show() 


#---------5Km/h différentes pentes-----------------(xi = 0.5, d = 8, A = 2, v = 5km/h)

# y_route2 = np.zeros(len(traj5[:,0]))

# for i in range(len(traj5[:,0])) : 
                        
#     if traj5[i,0] >= 0.5 and traj5[i,0] <= 8.5 :
    
#         y_route2[i] = 2*np.sin(2*np.pi*(traj5[i,0] - 0.5)/16)

# figure2 = plt.figure(figsize=(8,6))

# plt.subplot(2,1,1) 

# plt.plot(pente5 [:,0], pente5[:,1], label = "Trajectoire de la vendangeuse")
# plt.ylabel("y [m]")
# plt.xlim([0,15])
# plt.title("Influence de l'inclinaison du sol lors d'un changement de direction")

# plt.subplot(2,1,2)
# plt.plot(pente5[:,0], pente5[:,2]*180/np.pi, label = "Pente à 5%")
# plt.plot(pente8[:,0], pente8[:,2]*180/np.pi,label = "Pente à 8%")
# plt.xlim([0,15])
# plt.legend(loc = 'upper right')
# plt.xlabel("x [m]")
# plt.ylabel("Roulis [°]")
# figure2.savefig("plot/pentes.png")
# plt.show() 

# figure3 =  plt.figure(figsize=(7,2))

# plt.plot(pente5 [:,0], pente5[:,0]*5/100, label = "Pente à 5%" )
# plt.plot(pente5 [:,0], pente5[:,0]*8/100, label = "Pente à 8%" )
# plt.title("Inclinaison du sol")
# plt.xlim([0,15])
# plt.legend(loc = 'upper right')
# plt.xlabel("y [m]")
# plt.ylabel("z [m]")
# #plt.savefig("plot/sol_pente.png")
# plt.show() 


#--------Evitements à différentes vitesse-------------------

vit5 = np.loadtxt('vitesse5.res')
vit8 = np.loadtxt('vitesse8.res')
vit10 = np.loadtxt('vitesse10.res')

# y_route2 = np.zeros(len(vit5[:,0]))

# for i in range(len(vit5[:,0])) : 
                        
#     if vit5[i,0] >= 0.5 and vit5[i,0] <= 7.5 :
    
#         y_route2[i] = 2*np.sin(2*np.pi*(vit5[i,0] - 0.5)/14)


# figure4 = plt.figure(figsize=(12,6))

# plt.subplot(2,1,1) 
# plt.plot(vit5[:,0], vit5[:,1], label = "Trajectoire à 5 km/h")
# plt.plot(vit8[:,0], vit8[:,1], label = "Trajectoire à 8 km/h")
# plt.plot(vit10[:,0], vit10[:,1], label = "Trajectoire à 10 km/h")
# #plt.plot(vit5[:,0], y_route2,'--', label = "Trajectoire à suivre")

# plt.legend(loc = 'upper right')
# plt.ylabel("y [m]")
# plt.xlim([0,18])
# plt.title("Influence de la vitesse lors d'un changement de direction")

# plt.subplot(2,1,2)
# plt.plot(vit5[:,0], vit5[:,2]*180/np.pi, label = "Roulis à 5 km/h")
# plt.plot(vit8[:,0], vit8[:,2]*180/np.pi, label = "Roulis à 8 km/h")
# plt.plot(vit10[:,0], vit10[:,2]*180/np.pi, label = "Roulis à 10 km/h")
# plt.xlim([0,18])
# plt.legend(loc = 'upper right')
# plt.xlabel("x [m]")
# plt.ylabel("Roulis [°]")
# figure4.savefig("plot/vitesse.png")
# plt.show() 

#--------DORIAN-Tangage--------------------------------

# adapt_tang = np.loadtxt('tangage_active.res')
# tang = np.loadtxt('tangage_rien.res')

# # adapt_roulis = np.loadtxt('vitesse5.res')

# x = np.linspace(0,15, len(adapt_tang[:,0]))
# start = 5 #[m]
# A = 0.3    #[m]
# long = 8   #[m]
# condlist = [x<=start, x<start+long, x>=start+long]
# choicelist = [0, A*np.sin(np.pi*(x-start)/long), 0]
# sol = np.select(condlist, choicelist)

# figure5 = plt.figure(figsize=(8,6))

# plt.subplot(2,1,1)
# plt.plot(adapt_tang[:,0], sol, label="Bosse sur laquelle passe le véhicule")
# plt.legend(markerscale = 0.5, loc = 'upper right')
# plt.xlabel('Temps [s]')
# plt.ylabel('Hauteur [m]')

# plt.legend(loc = 'upper left')
# plt.ylabel("y [m]")
# plt.title("Efficacité de l'adaptation des vérins")

# plt.subplot(2,1,2)
# plt.plot(adapt_tang[:,0], adapt_tang[:,1], label = "Tangage avec adaptation des vérins")
# plt.plot(tang[:,0], tang[:,1], label = "Tangage sans adaptation des vérins")
# plt.legend(loc = 'upper left')
# plt.xlabel("x [m]")
# plt.ylabel("Tangage [°]")
# figure5.savefig("plot/tangage.png")
# plt.show() 


#--------DORIAN-Roulis--------------------------------

# adapt_roul = np.loadtxt('roulis_avec.res')
# roul = np.loadtxt('roulis_sans.res')

# # adapt_roulis = np.loadtxt('vitesse5.res')

# q = np.loadtxt("../resultsR/dirdyn_q.res")
# x = np.linspace(0,15, len(q[:,0]))
# start = 2 #[m]
# A = 0.3    #[m]
# long = 5   #[m]
# condlist = [x<=start, x<start+long, x>=start+long]
# choicelist = [0, A*np.sin(np.pi*(x-start)/long), 0]
# sol = np.select(condlist, choicelist)


# figure6 = plt.figure(figsize=(8,6))

# plt.subplot(2,1,1)
# plt.plot(roul[:,0], sol, label="Bosse sur laquelle passe le véhicule")
# plt.legend(markerscale = 0.5, loc = 'upper right')
# plt.xlabel('Temps [s]')
# plt.legend(loc = 'upper left')
# plt.ylabel("y [m]")
# plt.title("Efficacité de l'adaptation des vérins")

# plt.subplot(2,1,2)
# plt.plot(adapt_roul[:,0], adapt_roul[:,1], label = "Roulis avec adaptation des vérins")
# plt.plot(roul[:,0], roul[:,1], label = "Roulis sans adaptation des vérins")
# plt.legend(loc = 'upper left')
# plt.xlabel("x [m]")
# plt.ylabel("Roulis [°]")
# figure6.savefig("plot/roulis.png")
# plt.show() 

#--------DORIAN-tournant--------------------------------

adapt_evite = np.loadtxt('evite_avec.res')
evite = np.loadtxt('evite_sans.res')

# q = np.loadtxt("../resultsR/dirdyn_q.res") 
# x = q[:,1]
# start = 10 #[m]
# A     = 2    #[m]
# long  = 30   #[m]
# condlist = [x<=start, x<start+long, x>=start+long]
# choicelist = [0, A*np.sin(np.pi*(x-start)/long), 0]
# sol = np.select(condlist, choicelist)

y_route = np.zeros(len(evite[:,0]))

for i in range(len(traj[:,0])) : 
                        
    if evite[i,0] >= 10 and evite[i,0] < 39 :
    
        y_route[i] = 2*np.sin(2*np.pi*(traj[i,0] - 10)/60)


figure6 = plt.figure(figsize=(8,6))

plt.subplot(2,1,1)
plt.plot(traj[:,0], traj[:,1], label = "Trajectoire suivie par le véhicule")
plt.legend(markerscale = 0.5, loc = 'upper right')
plt.legend(loc = 'upper right')
plt.ylabel("y [m]")
plt.title("Efficacité de l'adaptation des vérins lors des virages")

plt.subplot(2,1,2)
plt.plot(adapt_evite[:,0], adapt_evite[:,1], label = "Roulis avec adaptation des vérins")
plt.plot(evite[:,0], evite[:,1], label = "Roulis sans adaptation des vérins")
plt.legend(loc = 'upper left')
plt.xlabel("x [m]")
plt.ylabel("Roulis [°]")
figure6.savefig("plot/roulis.png")
plt.show() 
