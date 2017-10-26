import serialIMU

myDevice = serialIMU.serialIMU('COM4',115200)

myDevice.streamIMUData();

while( True ):
	print(myDevice.getIMUData())