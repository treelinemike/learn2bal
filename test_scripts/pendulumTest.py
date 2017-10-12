# drawn largely from http://modelling3e4.connectmv.com/wiki/Software_tutorial/Integration_of_ODEs

import numpy as np
from scipy import integrate
import matplotlib.pyplot as plt
 
# link properties
rho = 2700                             # [kg/m^3] density (aluminum = 0.1 lbf/in^3 = 2700 kg/m^3)
l_link = 0.30                          # [m]
w_link = 0.07                          # [m]
h_link = 0.03                          # [m]
v = l_link * w_link * h_link           # [m^3]
m = rho * v                            # [kg]    (5kg = 11lb)
I = (m/12)*(4*(l_link**2)+(w_link**2)) # [kg*m^2]
g = 9.81                               # [m/s^2] 9.81m/s^2 on surface of earth
b = 0.08                               # [kg*m^2/s] damping constant

# derivative calcuation
# this funciton is called by ODE solver
def pendulumTestStateProp(t, X):
	"""
	Calculate Xdot at any time t
	"""
	
	# deconstruct state vector
	theta = X[0]
	theta_dot = X[1]
	
	# construct Xdot from differential equation
	# note: X = [theta theta_dot] therefore Xdot = [theta_dot theta_ddot]
	Xdot = np.zeros((2,1))
	Xdot[0] = theta_dot;
	Xdot[1] = -(m*g*l_link/(2*I))*np.sin(theta) -(b/I)*theta_dot;

	return Xdot
 
# main funciton for running ODE solver
if __name__ == '__main__':
 
	# integrator selection
	r = integrate.ode(pendulumTestStateProp).set_integrator('dopri5')
 
	# integration timeperiod
	t0 = 0.0
	tf = 10.0
	dt = 0.01
	nSteps = int(np.floor(((tf-t0)/dt)+1))
	
	# initial conditions
	X0 = np.zeros((2,1))
	X0[0] = 45    # degrees
	X0[1] = 0     # degrees/sec
	X0 = X0*np.pi/180
	r.set_initial_value(X0, t0)
 
	# data storage
	t = np.zeros((1,nSteps))
	X = np.zeros((2,nSteps))
	t[0] = 0
	X[:,0] = X0.T    # silly numpy matrix style requires that column to be inserted is actually a row...
 
	# integrate for each timestep and store output
	k = 1
	while r.successful() and k < nSteps:
		r.integrate(r.t + dt)
		t[0,k] = r.t
		X[:,k] = r.y.T
		k += 1
 
	# plot results using matplotlib
	fig = plt.figure(num=None,figsize=(10, 8))
	plt.subplot(211)
	plt.plot(t.T, X[0,:]*180/np.pi)
	plt.grid('on')
	plt.xlabel('Time [s]')
	plt.ylabel('Angle [deg]')
	plt.subplot(212)
	plt.plot(t.T, X[1,:]*180/np.pi)
	plt.grid('on')
	plt.xlabel('Time [s]')
	plt.ylabel('Angular Velocity [deg/s]')	
	fig.canvas.draw() 
	plt.show(block=True)  # setting block=False clears the figure and exits immedietly
	
	