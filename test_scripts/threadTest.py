import threading
import time
import logging

# global variables
totalCount = 0
endCounterThread = False
dataUpdateLock = threading.Lock()

# counting thread
# note: thread definition must appear above thread initialization
def myThread():
	global totalCount, endCounterThread
	while( endCounterThread != True ):
		
		# update count only once we can get a lock
		dataUpdateLock.acquire()  # blocks until lock available
		try:
			totalCount = totalCount + 1	
		finally:
			dataUpdateLock.release()
			
		# wait a bit
		time.sleep(0.5)

# initialize and start the counter thread
c1 = threading.Thread(name='counter1', target=myThread, daemon=True)
c1.start()

# main loop
while(1):
	
	# print totalCount as long as it is not currently being updated
	dataUpdateLock.acquire()
	try:
		thisCount = totalCount
	finally:
		dataUpdateLock.release()
	
	# kill counter thread at a certain point
	print(thisCount)
	if(thisCount > 5):
		endCounterThread = True
	
	# wait a bit
	time.sleep(1)