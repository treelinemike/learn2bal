import time

timerOverflow = True
dt = 0.05	

# initialize clock to current time
t0 = time.clock()

while(True):
	
	# do something useful here
	time.sleep(0.01)
	
	# then wait for end of the dt interval
	while( time.clock() < (t0 + dt)):
		# do nothing
		timerOverflow = False
	
	# fail if loop timing cannot be enforced
	assert( timerOverflow == False ), ("Code execution took too long!")
	
	# reset timer
	timerOverflow = True
	t0 = time.clock()