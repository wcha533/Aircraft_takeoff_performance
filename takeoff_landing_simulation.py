#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
Created on Sun Mar 17 14:27:33 2024

@author: wchanprasert
"""
# importing relavant library
import matplotlib.pyplot as plt
import numpy as np 
import math

# some functions
def roundup(x):
    return math.ceil(x / 2.0) * 2.0

# function that update sound speed based on an altitude
def updateSoundSp(h):
    g = 9.80665                             # [m/s2] gravitational acceleration
    rho_0 = 1.225                           # [kg/m3] air density at sea level
    temp_0 = 288.15                         # [K] temperature
    R = 287.05                              # air constant
    Lambda = -0.0065                        # [K/m] lapse rate
    temp = temp_0 + Lambda*h
    rho = (temp/temp_0)**(-g/Lambda/R-1)*rho_0
    soundSp = np.sqrt(1.4*R*temp)
    
    return soundSp,rho

# simulation case
interactiveMode = True
engineFail = True
abortTO = False
V_ef = 30
setT = 2
setD = 4

# constant 
# conversion factors
ftM = 0.3084                            # convert foot to metre
degRad = 2*np.pi                        # convert degree to radian

# ISA
g = 9.80665                             # [m/s2] gravitational acceleration
rho_0 = 1.225                           # [kg/m3] air density at sea level
temp_0 = 288.15                         # [K] temperature
p_0 = 101325                            # [N/m2] pressure
R = 287.05                              # air constant
Lambda = -0.0065                        # [K/m] lapse rate

# airfield information for take-off and landing calculation
hscr = 35 * ftM                         # [m] screen height
Hairp = 2000 * ftM                      # [m] airport altitude

# find atmospheric conditions at the airport
H = Hairp
# temp = temp_0 + Lambda*H
# rho = (temp/temp_0)**(-g/Lambda/R-1)*rho_0
# soundSp = np.sqrt(1.4*R*temp)
soundSp, rho = updateSoundSp(H)

# aircraft parameters
# aerodynamics
CD0 = 0.03                              # [-] zerp-drag coefficient
k = 0.045                               # [-] 1/(pi*A*e)
CD_spoiler = 0.0411                     # [-] Cd of the spoilers
CD_lg = 0.0243                          # [-] Cd of the landing gears
CL_max = 2.55                           # [-] max lift coefficient
CL_spoiler = -0.467                     # [-] Cl of the spoilers
CL_lg = 0                               # [-] Cl of the landing gears


# aircraft configuration and general data
nEngine = 2                             # [-] number of engine
m = 122470                              # [kg] aircraft mass
W = m*g                                 # [N] aircraft weight
S = 278.7                               # [m2] lift surface area
incAng = np.deg2rad(0.5)                # [rad] inclination angle of the turbofan
theta_fr = np.deg2rad(-1)               # [rad] aircraft pitch angle on ground - free roll  
theta_br = np.deg2rad(-1.5)             # [rad] aircraft pitch angle on ground - brake
mu_fr = 0.02                            # [-] tire friction coefficient - free roll
maxPitRate = np.deg2rad(5)              # [rad/s] max pitch rate by the pilot 5 deg/s
maxTheta = np.deg2rad(15)               # [rad] max aircraft pitch angle

V_ms = 0.94*np.sqrt(W/S*2/rho/CL_max)        # mininum stall airspeed at the airport altitude
V_2 = 1.2*V_ms      #1.2
V_mc = 0.9*V_2      #0.9
V_r = 1.05*V_mc #1.05                # setup rotation speed Vr based on V2 with in turn based on Vms

#V_r_range = np.linspace(40,60,2001)
#V_r_range = np.array([50.7])

# Turbofan engine data table at 2000 ft
#                 M, Idle gross thrust, gross Tmax TO, Idle drag, Dmax TO
engineData= np.array([[0  , 6672 , 211290, 0    , 0     ],
                      [0.1, 9786 , 216180, 5783 , 24470 ],
                      [0.2, 15124, 226300, 15124, 49820 ],
                      [0.3, 24465, 241980, 28913, 77100 ],
                      [0.4, 41368, 259780, 50710, 106760]])

# note that net thrust force is defined by Tnet = Tgross - Dmomentum

# simulation control 
dt = 0.01                               # time step size
digit = 2
    
# calculate initial condition
t = 0.0         
t = round(t, ndigits=digit)
theta = theta_fr
gamma = 0                               # [rad] initially set flight path angle equal to zero radian
alpha = theta - gamma  
CL_brake = 0                        
CL_basic = 0.6 + 4.8816*(alpha)
CL = CL_basic + CL_brake + CL_lg      # combined CL
V = 0                                   # [m/s] velocity
s = 0                                   # [m] traveled distance
x = s*np.cos(gamma)

CD_brake = 0
CD_basic = CD0 + k*CL_basic*CL_basic
CD = CD_basic + CD_brake + CD_lg      # combined CD

D = CD*0.5*rho*V*V*S*np.sign(V)         # [N] drag force
L = CL*0.5*rho*V*V*S                    # [N] lift force

M = V/soundSp
T = nEngine * (np.interp(M,engineData[:,0],engineData[:,2]) - np.interp(M,engineData[:,0],engineData[:,4]))

gammaRate = 0              

First = True
passVef = False

time = np.array([])
x_output = np.array([])
V_output = np.array([])
H_output = np.array([])
T_output = np.array([])
L_output = np.array([])
theta_output = np.array([])
gamma_output = np.array([])
alpha_output = np.array([])
    
time = np.append(time, t)
x_output = np.append(x_output, x)
V_output = np.append(V_output, V)
H_output = np.append(H_output, H)
T_output = np.append(T_output, T)
L_output = np.append(L_output, L)
theta_output = np.append(theta_output, theta)
gamma_output = np.append(gamma_output, gamma)
alpha_output = np.append(alpha_output, alpha)

#V_r_range = np.linspace(1.05*V_mc,1.4*V_mc,20)

#x_list = np.array([])

# check if engine in operation but there is a command to abort take-off 
if not engineFail and abortTO:
   print('Both Engines in operation, no aborted take-off\n')
   print('Braking system is overridden and take-off continues')
   abortTO = False
      
#for V_r in V_r_range:      
    
while True:          
        
   t = t + dt                           # time marching    
   t = round(t, ndigits=digit)   
   # update rho and speed of sound
   soundSp, rho = updateSoundSp(H)
   # Mach number
   M = V/soundSp
   
   # determine aircraft condition
   # Thrust, Friction coefficient, Spoilers deployment
   
   if not engineFail and V >= V_ef and not passVef:
      print('Engines 1 and 2 are in operation. You are good to go.')
      passVef = True
   
   if (engineFail and V >= V_ef) or (engineFail and passVef):
      
      
      if not passVef:
         if interactiveMode:
            print('Captain, you just lost one of your engine.')
            print('You have to make a decision now!')
            print('Abort take-off? [y/n]')
            pilotDecision = input()
            if pilotDecision == 'y':
               abortTO = True
            else:
               abortTO = False
               
         nEngine = 1   
         t_ef = t                # record engine failure time
         passVef = True          # flag the engine failure checkpoint
         
      if abortTO:
        
         if First:
            t_idleShift = t + 2                # set time to idle the engine by the pilot
            t_idleShiftEnd = t_idleShift + 5
            First = False
            T = nEngine * (np.interp(M,engineData[:,0],engineData[:,2]) - np.interp(M,engineData[:,0],engineData[:,4]))
        
         elif not First and t < t_idleShift:
            T = nEngine * (np.interp(M,engineData[:,0],engineData[:,2]) - np.interp(M,engineData[:,0],engineData[:,4]))
            
         elif t >= t_idleShift and t <= t_idleShiftEnd:           
            # the pilot starts to idle the engine, deploys spoilers and apply breaks 
            # engine spool down in 5 seconds     
            T_1 = nEngine * (np.interp(M,engineData[:,0],engineData[:,2]) - np.interp(M,engineData[:,0],engineData[:,4]))
            T_2 = nEngine * (np.interp(M,engineData[:,0],engineData[:,1]) - np.interp(M,engineData[:,0],engineData[:,3]))
            T_rate = (T_2 - T_1) / 5
                   
            CL_brake = CL_spoiler
            CD_brake = CD_spoiler
            
            T = T + T_rate * dt
        
            mu_fr = 0.63 - 0.0012*(V*1.94384)  # convert V from m/s to knots
            
         elif t > t_idleShiftEnd:
            CL_brake = CL_spoiler
            CD_brake = CD_spoiler
            T = nEngine * (np.interp(M,engineData[:,0],engineData[:,1]) - np.interp(M,engineData[:,0],engineData[:,3]))
            mu_fr = 0.63 - 0.0012*(V*1.94384)  # convert V from m/s to knots
     
      elif not abortTO:
         T = nEngine * (np.interp(M,engineData[:,0],engineData[:,setT]) - np.interp(M,engineData[:,0],engineData[:,setD]))
         CL_brake = 0
         CD_brake = 0
            
   else:       
             
      # control input thrust and pitch
      # update Thrust from the table     
      T = nEngine * (np.interp(M,engineData[:,0],engineData[:,setT]) - np.interp(M,engineData[:,0],engineData[:,setD]))
      CL_brake = 0
      CD_brake = 0
      
   if not abortTO:
      # pilot pullup maneuvering
      if V >= V_r:           
         #if theta >= maxTheta or gamma > 0:
         if theta >= maxTheta:
            pitRate = 0
         else:
            pitRate = np.deg2rad(3)       
      else:
         pitRate = 0

   else:
      pitRate = 0
         
   # calculate forces
   # first updating CL,CD
   alphaT = alpha + incAng
   CL_basic = 0.6 + 4.8816*(alpha)
   CL = CL_basic + CL_brake + CL_lg
   CD_basic = CD0 + k*CL_basic*CL_basic
   CD = CD_basic + CD_brake + CD_lg
   
   D = CD*0.5*rho*V*V*S*np.sign(V)
   L = CL*0.5*rho*V*V*S

   # Eq of Motion
   # m*a = T cos(alphaT) - D - Dg - W*sin(gamma)
   # m*V*d(gamma)/dt = L - W*cos(gamma) + T*sin(alphaT)
   # determine if on ground or airborne
   if W > L + T*np.sin(alphaT):
      # on ground
      N = W-L-T*np.sin(alphaT)
   else:
      # airborne
      N = 0
      # compute derivetive of gamma
      gammaRate = (L - W*np.cos(gamma) + T*np.sin(alphaT))/m/V
            
   #print(W - L + T*np.sin(alphaT), L/W)
   # compute derivetive terms from EoM
        
   a = (T*np.cos(alphaT) - D - W*np.sin(gamma) - mu_fr*N)/m           # compute acceleration from dV/dt = (T-D)/m
   
   # integration
   gamma = gamma + gammaRate*dt         # update gamma (fligh path angle) 
   
   
   if abortTO and passVef:
      if t >= t_idleShift:
         theta = theta_br
      
   else:
      theta = theta + pitRate*dt
   
   alpha = theta - gamma
  
   V = V + a*dt                         # update V
   x = x + V*dt*np.cos(gamma)           # update x
   H = H + V*dt*np.sin(gamma)
     
   # save data every time step
   time = np.append(time, t) 
   x_output = np.append(x_output, x)
   H_output = np.append(H_output, H)
   T_output = np.append(T_output, T)
   L_output = np.append(L_output, L)
   gamma_output = np.append(gamma_output, gamma)
   theta_output = np.append(theta_output, theta)
   alpha_output = np.append(alpha_output, alpha)
   V_output = np.append(V_output, V)
        
   if (H >= Hairp+hscr or V <= 0):
      # stop when the aircraft reaches the screnn height for take-off
      # OR V = 0 for aborted take-off
      break    
  
    
# plot position and velocity versus time    
fig, axs = plt.subplots(7, 1, figsize=(10, 10))
        
axs[0].plot(time, x_output)
axs[1].plot(time, V_output)
axs[2].plot(time, H_output/ftM)
axs[3].plot(time, np.rad2deg(gamma_output))
axs[4].plot(time, np.rad2deg(theta_output))
axs[5].plot(time, np.rad2deg(alpha_output))
axs[6].plot(time, T_output/1000)
        
#axs[0].set_title('Position')
#axs[1].set_title('Velocity')
#axs[2].set_title('Thrust')
       
# axs[0].set_xlabel('time [s]')
# axs[1].set_xlabel('time [s]')
# axs[2].set_xlabel('time [s]')
# axs[3].set_xlabel('time [s]')
# axs[4].set_xlabel('time [s]')
axs[6].set_xlabel('time [s]')

axs[0].set_yticks(np.linspace(0,3000,5))
axs[1].set_yticks(np.linspace(0,120,5))
axs[2].set_yticks(np.linspace(2000,2040,5))
axs[3].set_yticks(np.linspace(0,4,5))
axs[4].set_yticks(np.linspace(-5,20,6))
axs[5].set_yticks(np.linspace(-5,20,6))
axs[6].set_yticks(np.linspace(0,500,6))

for i in range(7):
    axs[i].set_xticks(np.arange(0,roundup(t)+1,2))

axs[0].set_xticklabels('')
axs[1].set_xticklabels('')
axs[2].set_xticklabels('')
axs[3].set_xticklabels('')
axs[4].set_xticklabels('')
axs[5].set_xticklabels('')
        
axs[0].set_ylabel('x [m]')
axs[1].set_ylabel('V [m/s]')
axs[2].set_ylabel('Altitude [ft]')
axs[3].set_ylabel('$\\gamma$ [deg]')
axs[4].set_ylabel('$\\theta$ [deg]')
axs[5].set_ylabel('$\\alpha$ [deg]')
axs[6].set_ylabel('T [kN]')

axs[2].axhline(hscr/ftM+Hairp/ftM,color='g')
axs[2].set_ylim([2000, 2040])

if not engineFail:
    axs[0].set_title('Normal Take-Off')
elif engineFail and not abortTO:
    axs[0].set_title('Continued Take-Off with an Engine Failure')
   
    for i in range(7):
      axs[i].axvline(t_ef,color='k')
elif engineFail and abortTO:
    axs[0].set_title('Aborted Take-Off with an Engine Failure')
    for i in range(7):
      axs[i].axvline(t_ef,color='k')
    
    for i in range(7):
      v1 = axs[i].axvline(t_idleShift,color='k')
      v1.set_linestyle('dashed')
      v2 = axs[i].axvline(t_idleShiftEnd,color='k')
      v2.set_linestyle('dotted')
   
# add grid
axs[0].grid()
axs[1].grid()
axs[2].grid()
axs[3].grid()
axs[4].grid()
axs[5].grid()
axs[6].grid()
        
# show the plots
plt.show()