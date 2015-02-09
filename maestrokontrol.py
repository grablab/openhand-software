from nanokontrol import *
from lib_pololu import *

class MaestroKontrol(NanoKontrol):
	N = 0	#number of servos (need to be hooked up in series)
	Maestro = None

	def __init__(self,N,port="/dev/ttyACM0",midi_id=None):
		NanoKontrol.__init__(self,midi_id)
		self.N = N
		self.Maestro = Maestro(port)
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
						
						if control_id < 8 and control_id < self.N:	#only looking at slider values:
							val = float(control_val) / self.MAX_NANO_VALUE
							print "Servo id "+repr(control_id)+": "+repr(val)
							self.Maestro.moveServo(control_id,val)
		except KeyboardInterrupt:
			pass

if __name__=="__main__":
	N = int(sys.argv[1])
	port = sys.argv[2]

	M = MaestroKontrol(N,port)
	
	
