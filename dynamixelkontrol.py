from nanokontrol import *
from lib_robotis_mod import *

class DynamixelKontrol(NanoKontrol):
	dyn_ids = None
	dyn = None
	servos = None
	modes = None

	MAX_TORQUE = 1.0	#safety value for Dynamixel
	DYN_MODEL = "RX"	#can also be MX
	
	def __init__(self,port,dyn_ids,dyn_model="RX",midi_id=None):
		NanoKontrol.__init__(self,midi_id)
		self.DYN_MODEL = dyn_model
		self.dyn_ids = dyn_ids
		if len(self.dyn_ids)>8:
			print "Warning: Only initial 8 servos will be controlled"
		
		self.dyn = USB2Dynamixel_Device(port)
		
		#generate dynamixels:
		self.servos = [None]*len(self.dyn_ids)
		self.modes = [True]*len(self.dyn_ids)	#true is position mode

		for i in xrange(len(self.dyn_ids)):
			self.servos[i] = Robotis_Servo(self.dyn,self.dyn_ids[i],self.DYN_MODEL)	
			self.servos[i].apply_speed(1.0)
			self.servos[i].apply_max_torque(self.MAX_TORQUE)
			self.servos[i].move_to_encoder(0)
		self.start()
		
	def moveServo(self,index,val):
		self.servos[index].move_to_encoder(int(val*self.servos[index].settings['max_encoder']))

	def torqueServo(self,index,val):
		val = min(1.0,max(val,0))
		s = self.servos[index]
		s.apply_max_torque(val*0.25)
		enc = s.read_encoder()
		target_enc = int(min(s.settings['max_encoder'],enc+s.settings['max_encoder']*0.2))
		s.move_to_encoder(target_enc)
	
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
						
						if control_id < 8 and control_id < len(self.servos):	#only looking at slider values:
							if self.modes[control_id]:
								self.moveServo(control_id,float(control_val)/self.MAX_NANO_VALUE)
							else:
								self.torqueServo(control_id,float(control_val)/self.MAX_NANO_VALUE)
						if control_id >= 48 and control_id <56 and control_id <48+len(self.servos) and control_val>0:	#toggle control mode
							servo_id = control_id-48
							self.modes[servo_id] = not self.modes[servo_id]
							if self.modes[servo_id]:
								print "Switching servo "+repr(servo_id)+" to position mode"
								self.servos[servo_id].apply_max_torque(self.MAX_TORQUE)
								self.servos[servo_id].move_to_encoder(self.servos[servo_id].read_encoder())
							else:
								print "Switching servo "+repr(servo_id)+" to torque mode"
								self.torqueServo(servo_id,0)
								
						if control_id == 41 and control_val>0:
							for s in self.servos:
								print "Servo "+repr(s.servo_id)+": "+repr(s.read_encoder())+"/"+repr(s.settings["max_encoder"])+", "+repr(float(s.read_encoder())/s.settings["max_encoder"])
					
				#time.sleep(0.05)
		except KeyboardInterrupt:
			pass
			
if __name__== "__main__":
	port = sys.argv[1]
	dyn_model = sys.argv[2]
	dyn_ids = [0]*(len(sys.argv)-3)

	for i in xrange(len(dyn_ids)):
		dyn_ids[i] = int(sys.argv[i+3])
	
	d = DynamixelKontrol(port,dyn_ids,dyn_model)	#may need to append the midi id here if errors are prevalent
