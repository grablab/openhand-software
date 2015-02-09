import serial
import sys

#Maestro controller that uses the Pololu protocol
class Maestro():
	ser = None
	servoMin = 528
	servoMax = 2464
	reverse = True
	
	def __init__(self,port=None):
		if port==None:
			self.ser = serial.Serial('/dev/ttyACM0')
		else:
			self.ser = serial.Serial(port)
		self.reset()

	def reset(self):
		self.ser.write(chr(0xA1))
		self.ser.write(chr(0xA2))

	def setSpeed(self,n,val):
		if val<0 or val>1:
			val = min(max(val,0),1)
			print "ERROR: Value restricted to range [0,1]"
		target = int(val*127)
		lo = target & 127
		hi = (target >> 7) & 127
		bud = chr(0x87)+chr(int(n))+chr(lo)+chr(hi)
		self.ser.write(bud)

	def moveServo(self,n,val):
		if val<0 or val>1:
			val = min(max(val,0),1)
			print "ERROR: Value restricted to range [0,1]"
		if self.reverse:
			target = int(self.servoMax-val * (self.servoMax-self.servoMin))*4
		else:
			target = int(val * (self.servoMax-self.servoMin)+self.servoMin)*4
		
		lo = target & 127
		hi = (target >> 7) & 127
		bud = chr(0x84)+chr(int(n))+chr(lo)+chr(hi)
		self.ser.write(bud)

	def moveServos(self,n_arr,val_arr):
		if len(n_arr)!=len(val_arr):
			print "ERROR: Mismatch between channel and target arrays"
		else:
			bud = chr(0x9f)+chr(len(n_arr))+chr(n_arr[0])
		for i in xrange(len(n_arr)):
			n = n_arr[i]
			val = val_arr[i]
			target = int(val * (self.servoMax-self.servoMin)+self.servoMin)*4
			lo = target & 127
			hi = (target >> 7) & 127
			
			bud = bud+chr(lo)+chr(hi)
		self.ser.write(bud)

	def readServo(self,n):
		bud = chr(0x90)+chr(n)
		self.ser.write(bud)
		lsb = ord(self.ser.read())
		msb = ord(self.ser.read())

		return (((msb << 8) + lsb) / 4 - self.servoMin) / (self.servoMax-self.servoMin)
if __name__=="__main__":
	if len(sys.argv)>1:
		port = sys.argv[1]
	else:
		port = None

	M = Maestro(port)
