from nanokontrol import *
from lib_robotis_mod import *
from openhand import *

class DynamixelHandKontrol(NanoKontrol):
	hand = None

	def __init__(self,hand,midi_id=None):
		NanoKontrol.__init__(self,midi_id)
		self.hand = hand
		self.start()
		
	def start(self):
		try:
			while(True):
				data = self.controller.read(1)
				for event in data:
					control = event[0]
					timestamp = event[1]
					
					if (control[0] & 0xF0) == 176:
						control_id = control[1] | ((control[0] & 0x0F) << 8)
						control_val = control[2]
						
						if control_id < 8 and control_id < len(self.hand.servos):	#only looking at slider values:
							self.hand.moveMotor(control_id,float(control_val)/self.MAX_NANO_VALUE)
						if control_id == 41 and control_val>0:
							self.hand.diagnostics()
				#time.sleep(0.05)
		except KeyboardInterrupt:
			pass

def checkServos(expected,received):
	if received!=expected:
		print "Expected "+repr(expected)+" servo ids, got "+repr(received)
		return False
	else:
		return True
		
if __name__== "__main__":
	port = sys.argv[1]
	dyn_model = sys.argv[2]
	hand_model = (sys.argv[3]).lower()
	dyn_ids = [0]*(len(sys.argv)-4)

	Hand = None

	for i in xrange(len(dyn_ids)):
		dyn_ids[i] = int(sys.argv[i+4])

	#check for valid hand models:
	if hand_model=="model_t":
		if checkServos(1,len(dyn_ids)):
			Hand = Model_T(port,dyn_ids[0],dyn_model)	
	elif hand_model=="model_t42":
		if checkServos(2,len(dyn_ids)):
			Hand = Model_T42(port,dyn_ids[0],dyn_ids[1],dyn_model)
	elif hand_model=="model_o":	#order: ab/add, forward finger, reverse finger, thumb
		if checkServos(4,len(dyn_ids)):
			Hand = Model_O(port,dyn_ids[0],dyn_ids[1],dyn_ids[2],dyn_ids[3],dyn_model)
	elif hand_model=="model_q":	#order: central pivot, finger 1, finger 2, middle pair
		if checkServos(4,len(dyn_ids)):
			Hand = Model_Q(port,dyn_ids[0],dyn_ids[1],dyn_ids[2],dyn_ids[3],dyn_model)
	elif hand_model=="gpp":
		if checkServos(2,len(dyn_ids)):
			Hand = GripperPP(port,dyn_ids[0],dyn_ids[1],dyn_model)
	elif hand_model=="twiddler":
		if checkServos(2,len(dyn_ids)):
			Hand = Twiddler(port,dyn_ids[0],dyn_ids[1],dyn_model)

	if Hand is None:
		print "[ERR] Invalid hand"
	else:
		DynamixelHandKontrol(Hand)
		#initialize dynamixel controller for specified hand
