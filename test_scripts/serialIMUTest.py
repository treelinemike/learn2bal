import serialIMU

myDevice = serialIMU.serialIMU('COM4',115200)

while( True ):
	print(myDevice.getIMUData())