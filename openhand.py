#!/usr/bin/python -i
from lib_robotis_mod import *
import time
import numpy as np	#for array handling

#assumptions:
	#only dynamixel servos being used
	#either all RX or MX servos being used (no mixing)
	#different encoder limits for each type
		#motor limits and mov't designated in terms of proportion, not encoder value
class OpenHand():
	dyn = None
	port = None
	servo_ids = []
	servos = []
	
	motorDir = []
	motorMin = []
	motorMax = []
	
	pos_release = 0.05
	pos_close = 0.5

	pause = 1.0		#amount of time to wait for move commands

	HAND_HEIGHT = 0.14	#hand height from base to palm (m) used for arm IK and approach vectors
	WRIST_OFFSET = -np.pi/3	#J5 offset (rad) to accommodate final orientation

	def __init__(self,port,servo_ids,series="RX"):
		self.port = port
		self.dyn = USB2Dynamixel_Device(port)	#always only one
		self.servo_ids = servo_ids
		for servo_id in self.servo_ids:
			self.servos.append(Robotis_Servo(self.dyn,servo_id,series))
		for servo in self.servos:		#make sure we're in position mode
			servo.kill_cont_turn()

	def reset(self):	#returns everything to zeroed positions, different from release
		print "ERROR: reset() not implemented]n"
		return False

	def release(self):	#opens the finger components, doesn't necessarily move non-finger servos
		print "ERROR: release() not implemented\n"
		return False

	#close functions are normalized regardless of mode such that the operating range [0,1.0] makes sense 
	def close(self,amnt=0.5):
		print "ERROR: close() not implemented\n"
		return False

	#move servo according to amnt, not encoder value
	def moveMotor(self,index,amnt):
		if amnt < 0. or amnt > 1.0:
			print "WARNING: motion out of bounds, capping to [0,1]. Index: "+repr(index)+", Cmd:"+repr(amnt)
		amnt = min(max(amnt,0.),1.0)

		if (index < 0 or index >= len(self.servos)):
			print "ERROR: invalid motor index "+repr(index)
		else:
			servo = self.servos[index]
			if self.motorDir[index]>0:	#normal case
				servo.move_to_encoder(int(servo.settings["max_encoder"]*(self.motorMin[index] + amnt*(self.motorMax[index]-self.motorMin[index]))))
			else:				#reverse
				servo.move_to_encoder(int(servo.settings["max_encoder"]*(self.motorMax[index] - amnt*(self.motorMax[index]-self.motorMin[index]))))
	def preventAllLoadErrors(self):
		for i in range(len(self.servos)):
			self.preventLoadError(i)
	def preventLoadError(self,i):
		if abs(self.servos[i].read_load()) > 80:	#arbitrary load threshold
			self.servos[i].move_to_encoder(self.servos[i].read_encoder())

	def diagnostics(self):
		for servo in self.servos:
			print "---"
			print "Servo ID: "+repr(servo.servo_id)
			print "Load: "+repr(servo.read_load())
			print "Temperature: "+repr(servo.read_temperature())
			print "Target Encoder: "+repr(servo.read_target_encoder())
			print "Current Encoder: "+repr(servo.read_encoder())

class Model_Q(OpenHand):
	servo_speed = 0.25
	
	pos_close = 0.35

	motorDir = [1,-1,1,1]
	motorMin = [0.32,0.4,0.05,0.05]
	motorMax = [0.67,0.9,0.5,0.5]

	HAND_HEIGHT = 0.12
	WRIST_OFFSET = -np.pi/4

	def __init__(self,port="/dev/ttyUSB0",s1=13,s2=12,s3=11,s4=10):
		#s1: internal twist between fingers
		#s2: adaptive two-finger power-grasp
		#s3: single finger 1 (usually the forefinger)
		#s4: single finger 2 (usually the thumb or shorter finger if implemented as such)
		OpenHand.__init__(self,port,[s1,s2,s3,s4],"MX")
		for servo in self.servos:
			servo.apply_speed(self.servo_speed)
			servo.apply_max_torque(0.4)
		self.reset()

	#default OpenHand commands
	def release(self):
		#self.reset()
		self.moveMotor(1,0.)
		self.moveMotor(2,0.)
		self.moveMotor(3,0.)
		time.sleep(self.pause)
		self.preventAllLoadErrors()
	def close(self,amnt=0.35):
		self.moveMotor(1,amnt)
		self.moveMotor(2,amnt)
		self.moveMotor(3,amnt)
		time.sleep(self.pause)
		self.preventAllLoadErrors()
	def reset(self):
		self.moveMotor(0,0.5)
		self.moveMotor(1,0.)
		self.moveMotor(2,0.)
		self.moveMotor(3,0.)
		time.sleep(self.pause)
		self.preventAllLoadErrors()

	#model-specific commands
	def closeFF(self,amnt):
		self.moveMotor(1,amnt-0.1)
		time.sleep(self.pause)
		self.preventLoadError(1)

	def closeFP(self,amnt):
		self.moveMotor(2,amnt)
		self.moveMotor(3,amnt)
		time.sleep(self.pause)
		self.preventLoadError(2)
		self.preventLoadError(3)

	def twist(self,direction=1,amnt=0.5,num=5):
		num = max(0,num)
		self.closeFP(self.pos_release)
		for i in range(0,num):
			self.closeFF(amnt)
			self.closeFP(self.pos_release)
			if direction>0:
				self.moveMotor(0,0.)
				time.sleep(self.pause)
				self.preventLoadError(0)
				self.closeFP(amnt)
				self.closeFF(0.)
				self.moveMotor(0,1.)
			else:
				self.moveMotor(0,1.)
				time.sleep(self.pause)
				self.preventLoadError(0)
				self.closeFP(amnt)
				self.closeFF(0.)
				self.moveMotor(0,0.)
			time.sleep(self.pause)
			self.preventLoadError(0)

	def pinchPower(self,pinchAmnt=0.5,offsetAmnt=0.2,num=3):
		num = max(0,num)
		self.closeFP(self.pos_release)
		for i in range(num):
			self.closeFF(pinchAmnt)
			self.closeFP(self.pos_release)	#finger replacement/recycle
			self.closeFP(pinchAmnt)		#begin grasp exchange
			self.closeFF(self.pos_release)
			self.closeFP(pinchAmnt+offsetAmnt)	#pull finger closer inward
		self.closeFF(pinchAmnt)
		self.closeFP(self.pos_release)
		self.closeFP(pinchAmnt)

class Model_T42(OpenHand):
	servo_speed = 0.25

	motorDir = [1,1]
	motorMin = [0.,0.]
	motorMax = [1.,1.]

	HAND_HEIGHT = 0.1
	WRIST_OFFSET = -np.pi/4

	def __init__(self,port="/dev/ttyUSB0",s1=1,s2=2):
		#s1: "forefinger"
		#s2: "thumb"
		OpenHand.__init__(self,port,[s1,s2],"RX")
		self.encoder_release = 50
		for servo in self.servos:
			servo.apply_speed(self.servo_speed)
		self.reset()

	#default OpenHand commands:
	def reset(self):
		self.moveMotor(0,0.)
		self.moveMotor(1,0.)		

	def close(self,amnt=0.5):	#position-based closing mechanism
		self.moveMotor(0,amnt)
		self.moveMotor(1,amnt)

	def release(self):
		self.moveMotor(0,self.pos_release)
		self.moveMotor(1,self.pos_release)

	#model-specific OpenHand commands:
	def flip_init(self):
		self.moveMotor(0,self.pos_release)
		self.moveMotor(1,self.pos_close)

	def flip_close(self):
		self.moveMotor(0,self.pos_close)
		self.moveMotor(1,self.pos_close)
		

class Model_T(OpenHand):
	motorDir = [1]
	motorMin = [0.]
	motorMax = [1.]

	max_torque = 0.2	#determining max torque in MX series is tricky...this value should only be used in torque-close

	HAND_HEIGHT = 0.14
	WRIST_OFFSET = -np.pi/4

	def __init__(self,port="/dev/ttyUSB0",s1=1):
		OpenHand.__init__(self,port,[s1],"MX")
		self.reset()

	def reset(self):
		self.moveMotor(0,self.pos_release)

	def close(self,amnt=0.5):
		self.servos[0].apply_max_torque(1.0)	#note that apply torque and apply max_torque are COMPLETELY DIFFERENT =)
		self.servos[0].kill_cont_turn()
		self.servos[0].enable_torque_mode()
		self.servos[0].apply_torque(amnt*self.max_torque)
		time.sleep(0.5)

		while True:
			sp = self.servos[0].read_speed()
			if (sp <= 0):
				self.servos[0].disable_torque_mode()
				p = self.servos[0].read_encoder()
				self.servos[0].move_to_encoder(p)
				break
			else:
				print "close (speed): "+repr(sp)
				time.sleep(0.5)
		return True
	
	def close_wheel(self,amnt=0.5,speed=0.2):	#closing through wheel mode
		#set torque output to max, use wheel speed to modulate closing force
		self.servos[0].disable_torque_mode()
		self.servos[0].apply_max_torque(amnt)
		self.servos[0].init_cont_turn()
		self.servos[0].apply_speed(speed)
		time.sleep(0.5)

		while True:
			sp = self.read_speed()
			if (sp <= 0):
				self.servos[0].kill_cont_turn()
				p = self.servos[0].read_encoder()
				self.servos[0].apply_max_torque(1.0)
				self.servos[0].move_to_encoder(p)
				break
			else:
				print "close (speed): "+repr(sp)
				time.sleep(0.5)

	def close_pos(self,amnt=0.2):			#closing through position mode and torque limit
		#set target position to furthest limit, but change servo torque limit
		self.servos[0].kill_cont_turn()
		self.servos[0].disable_torque_mode()
		self.servos[0].apply_max_torque(amnt)
		self.servos[0].move_to_encoder(self.settings['max_encoder']-1)
		time.sleep(0.5)
		
		while True:	
			sp = self.read_speed()	
			if (sp <= 0):
				p = self.servos[0].read_encoder()
				self.servos[0].apply_max_torque(1.0)
				self.servos[0].move_to_encoder(p)
				break
			else:
				print "close (speed): "+repr(sp)
				time.sleep(0.5)

	def release(self):		#should work for all previous close cases
		self.servos[0].kill_cont_turn()
		self.servos[0].apply_max_torque(1.0)	
		self.servos[0].enable_torque_mode()

		self.servos[0].apply_torque(0.)			#allow natural compliance to loosen grasp for tight grip cases
		time.sleep(2.)

		self.servos[0].disable_torque_mode()
		self.servos[0].apply_speed(1.0)	#check in case it was in wheel mode
		self.moveMotor(0,self.pos_release)	
