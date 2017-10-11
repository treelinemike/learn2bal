import numpy as np
import serialIMU as simu
import time
import threading

# global variables and settings
dt = 0.02    						          # loop period
endObsThread = False		                  # flag to kill measurement thread
freshIMUData = False                          # flag to identify when new IMU data is available
obsUpdateLock = threading.Lock()              # measurement data update lock
latestObs = np.asmatrix(np.zeros((3,1)))      # latest observation from IMU

# observation update thread
# keeps main thread from having to block on IMU read
def obsThread():
	global totalCount
	global endObsThread
	global freshIMUData
	global obsUpdateLock
	
	# initialize serial connection to IMU
	imu = simu.serialIMU('COM4',115200)
	
	# update observations as fast as we can
	while( endObsThread != True ):
		
		# get current time
		tobs1 = time.clock()
		
		# update count only once we can get a lock
		rawIMUData = imu.getIMUData()
		lockStatus = obsUpdateLock.acquire(timeout=1)  # blocks until lock available
		if( lockStatus == False ):
			endObsThread = True
			assert( False ), ('Observation update thread failed to obtain lock')
		try:
			latestObs[0] = rawIMUData[3]*9.81            # a_x      [m/s^2]
			latestObs[1] = rawIMUData[4]*9.81            # a_y      [m/s^2]
			latestObs[2] = rawIMUData[2]*(np.pi/180)     # omega_z  [rad/s^2]
			freshIMUData = True
		finally:
			obsUpdateLock.release()
		
		# force update at 50 Hz
		while( time.clock() < (tobs1 + 0.001)):
			time.sleep(0.001) # pass doesn't work here for whatever reason
		
# initialize state vector to all zeros
xk_prev = np.asmatrix(np.zeros((6,1)))

# prepare state transition matrix
stateTrans = dt*np.asmatrix(np.eye(6))
stateTrans[2,2] = 1
stateTrans[5,5] = 1

# prepare state transition jacobian
F = dt*np.asmatrix(np.zeros((6,6)))
F[0,1] = dt
F[1,2] = dt
F[2,2] = 1
F[3,4] = dt
F[4,5] = dt
F[5,5] = 1

# state error mapping  TODO: FIX PLACEHOLDER
G = np.asmatrix(np.eye(6))

# state covariance matrix  TODO: FIX PLACEHOLDER
Q = np.asmatrix(np.zeros((6,6)))

# error covariance matrix  TODO: FIX PLACEHOLDER
Pk_prev = np.asmatrix(np.zeros((6,6)))		

# catching KeyboardInterrupt to terminate with CTRL-C
try:
	# start observation update thread
	obsUpdater = threading.Thread(name='obsUpdater1', target=obsThread)
	obsUpdater.start()
	time.sleep(1)

	# repeat indefinitely
	while( True ):
		# reset timer
		timerOverflow = True
		t0 = time.clock()
		
		# wait if no new IMU data is available
		# hopefully this loop will never execute
		while( freshIMUData == False ):
			time.sleep(0.001) # pass doesn't work here for whatever reason

		# get latest observation
		lockStatus = obsUpdateLock.acquire(timeout=1)  # blocks until lock available
		if( lockStatus == False ):
			endObsThread = True
			assert( False ), ('Main loop failed to obtain observation update lock')
		try:
			zk = latestObs
			freshIMUData = False
		finally:
			obsUpdateLock.release()
		
		# define forward observation model and 
		hk = np.matrix([[1,2],[3,4]])
		Hk = 0
		
		# predict step: propigate 'dynamics' forward in time
		xk_prior = stateTrans*xk_prev
		Pk_prior = F*Pk_prev*F.transpose() + G*Q*G.transpose()
		
		# correct step -- TODO: THESE ARE ALL PLACEHOLDERS!
		K = Pk_prior  
		xk_posterior = xk_prior
		Pk_posterior = Pk_prior
		
		# prepare for next step
		# prepare for next step
		xk_prev = xk_posterior
		Pk_prev = Pk_posterior

		# enforce loop timing to period of length dt
		while( time.clock() < (t0 + dt)):
			timerOverflow = False
		
		# fail if loop timing cannot be enforced
		# note: won't detect a long call to serialIMU.getIMUData() until that function actually returns; could be hung for a while before we realize it
		if( timerOverflow == True ):
			endObsThread = True
			assert( False ), ("Code execution took too long!")
			
# exit on KeyboardInterrupt
except KeyboardInterrupt:
	endObsThread = True
	print('Terminating due to KeyboardInterrupt')
	exit()
