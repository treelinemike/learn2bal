import sys				# for streming to stdout
import serial			# pySerial for serial port functionality
import struct			# for data conversion from binary 

# recommended usage:
# >> import serialIMU
# >> myDevice = serialIMU.serialIMU('COM4',115200)
# >> myDevice.getIMUData()
# >> myDevice.closeSerialPort()

class serialIMU:
	
	# serial control character definitions
	DLE_byte = 0x10
	STX_byte = 0x02
	ETX_byte = 0x03
	
	# class variables 
	byteBuffer = bytearray()   # list to hold raw bytes read from serial line
	firstRun = True			   # flag indicating first message processed (for starting time t)
	microTime = 0              # time reported by the microcontroller
	t = 0                      # time since first packet read (i.e. time for plotting signals)
	dispCount = 0              # counter for displaying data on screen
	dispEvery = 10             # display data once every xx messages
	
	# convert raw two's complement IMU output into appropriate floating point values
	def imuConvert(self, data, range ):

		# convert data out of bytes
		# TODO: this is pathetically inelegant
		data = bytearray(data)
		data.extend(b'\x00\x00')
		data = struct.unpack('<L',data)
		data = data[0]
		
		# deal with negative numbers
		if ((data & (1 << 15)) != 0):
			imuVal = (~(data-1) & ((1 << 16)-1))
			imuVal = imuVal*(-1*range/(((1 << 16)/2)))
			
		# non-negative numbers
		else:
			imuVal = data*(range/(((1 << 16)/2)-1))
			
		return imuVal
	
	# read a single message from the IMU via the established serial link
	# note: serial buffer likely needs to be flushed before calling this for the first time
	# flushing is not included in this function as it would disrupt continuous streaming
	def readIMUMessage(self):
	
		# flag to identify when we've received a valid message
		validMessage = False
		
		# parse serial data until a valid message is received
		while ( validMessage != True ):
		
			# read one byte from the serial line
			# this value can be increased slightly, but larger reads result in significant lag
			data = self.ser.read(1)
			dataLen = len(data)
			
			# add these bytes to the END of the serial bytes vector
			if( dataLen > 0 ):
				self.byteBuffer.extend(bytearray(data))

				# look for the start of a message
				while( (len(self.byteBuffer) > 1) and (self.byteBuffer[0] != self.DLE_byte) and (self.byteBuffer[1] != self.STX_byte) ):
					if ((self.byteBuffer[0] == self.DLE_byte) and (self.byteBuffer[1] == self.DLE_byte)):
						self.byteBuffer.pop(0)
						self.byteBuffer.pop(0)
					else:
						self.byteBuffer.pop(0)
				
				# if there are at least two bytes left in the buffer, we should be at the start of a message
				if( len(self.byteBuffer) > 2 ):
				
					# message search variables that will be used later
					# within the scope of this if() statement
					msgFound = False
					fwdIdx = 2
								
					# look for the end of a message
					# TODO: this needs to be optimized!
					while ( (msgFound == False) and (fwdIdx < len(self.byteBuffer)) ):
						
						# is our fwdIdx marker at an ETX_byte?
						if ( self.byteBuffer[fwdIdx] == self.ETX_byte ):

							# if so, count the number of (DLE_byte) characters
							# that immediately preceed it
							atPrevDLE = True
							numPrevDLE = 0
							revIdx = fwdIdx - 1
							while ( atPrevDLE and (revIdx > 0) ):
								if (self.byteBuffer[revIdx] == self.DLE_byte):
									numPrevDLE += 1
									revIdx -= 1
								else:
									atPrevDLE = False
							
							# if we found an odd number of previous DLEs, this is a message
							if ( (numPrevDLE > 0) and ((numPrevDLE % 2) != 0) ):
								msgFound = True

						# increment counter to move on to next byte
						fwdIdx += 1			 
																		 
					# if we found a message, extract it into its own structure
					# message will start at byteBuffer[0] and end at byteBuffer[fwdIdx-1]
					# because we incremented fwdIdx at the end of the while() loop above
					if ( msgFound == True ):
						
						# new bytearray to contain this message 
						newMsg = bytearray()
						
						# copy the message found into its own bytearray
						copyIdx = 0
						while ( copyIdx < fwdIdx ):
							# removed stuffed DLE_byte characters
							if ( (len(self.byteBuffer) > 1) and (self.byteBuffer[0] == self.DLE_byte) and (self.byteBuffer[1] == self.DLE_byte) ):
								self.byteBuffer.pop(0)
								copyIdx += 1
						
							# we should always have a byte left in the buffer here
							# so this assertion should never be raised
							assert (len(self.byteBuffer) >= 0), ("Big Error: 0 serial bytes", len(self.byteBuffer))
							
							# add current character to the new message
							newMsg.append(self.byteBuffer.pop(0))
							copyIdx += 1;

						# save only messages of the proper length
						# TODO: check the packet type, and the CRC... for now we go on faith that the correct length packet is right...
						if (len(newMsg) != 23):
							newMsg.clear()		

						# if message is valid, process it appropriately
						else:

							validMessage = True
						
							# extract microcontroller time
							# TODO: this is pathetically inelegant
							microTimeBytes = newMsg[3:5]
							microTimeBytes.extend(b'\x00\x00')
							newMicroTime = struct.unpack('<L',microTimeBytes)
							newMicroTime = newMicroTime[0]
							
							# update elapsed time for plotting signals
							if( self.firstRun == True ):
								self.microTime = newMicroTime
								self.firstRun = False
							else:
								if (newMicroTime > self.microTime):
									self.t += (newMicroTime - self.microTime)*(0.000016)
									self.microTime = newMicroTime
								else:
									self.t += ((65535-self.microTime) + newMicroTime + 1)*(0.000016)
									self.microTime = newMicroTime
						
							# finally, process the accelerometer and gyro signals per ST axes
							Gy = self.imuConvert(newMsg[8:10], 1000.0)
							Gx = -1*self.imuConvert(newMsg[10:12], 1000.0)
							Gz = self.imuConvert(newMsg[12:14], 1000.0)
							Ay = self.imuConvert(newMsg[14:16], 2.0)
							Ax = -1*self.imuConvert(newMsg[16:18], 2.0)
							Az = self.imuConvert(newMsg[18:20], 2.0)
							
							# return 
							return (Gx,Gy,Gz,Ax,Ay,Az)
							
	# get data from the serial IMU once
	def getIMUData(self):
		
		# clear serial input and output buffers
		self.ser.reset_input_buffer()
		self.ser.reset_output_buffer()
	
		return self.readIMUMessage()
	
	# stream IMU data to stdout in real time
	def streamIMUData(self):
	
		# clear serial input and output buffers
		self.ser.reset_input_buffer()
		self.ser.reset_output_buffer()
	
		while( True ):
			 thisMsg = self.readIMUMessage()
			 sys.stdout.write("%+8.2f   %+8.2f   %+8.2f   %+5.2f   %+5.2f   %+5.2f\r" % thisMsg)

	# open the serial port
	def openSerialPort(self):
		self.ser = serial.Serial(port=self.port,baudrate=self.baud,bytesize=serial.EIGHTBITS, parity=serial.PARITY_NONE, stopbits=serial.STOPBITS_ONE,timeout=0.0005)

	# close the serial port
	def closeSerialPort(self):
		self.ser.close()
	
	# constructor: requires both a serial port and a baud rate
	def __init__(self,serialPort,baudRate):
		self.port = serialPort
		self.baud = baudRate
		self.openSerialPort()