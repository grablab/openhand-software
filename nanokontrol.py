#korg controller - based off of ros-drivers/korg_nanokontrol

import pygame
import pygame.midi
import signal

import sys
import time
import os

class NanoKontrol():
	controller = None
	MAX_NANO_VALUE = 128
	
	def __init__(self,id=None):
		if id==None:
			if os.name=='nt':
				id=1	#windows condition
			elif os.name=='posix':
				id=3	#linux condition
	
		pygame.midi.init()
		devices = pygame.midi.get_count()
		if devices<1:
			print "No MIDI devices detected"
			exit(-1)
		print "Found %d MIDI devices" % devices
		
		if id is not None:
			input_dev = int(id)
		else:
			input_dev = pygame.midi.get_default_input_id()
			if input_dev==-1:
				print "No default MIDI input device"
				exit(-1)
		print "Using input device %d" % input_dev
		
		self.controller = pygame.midi.Input(input_dev)
		
	def _poll(self):
		data = self.controller.read(1)
		for event in data:
			control = event[0]
			timestamp = event[1]
			
			if (control[0] & 0xF0) == 176:
				control_id = control[1] | ((control[0] & 0x0F) << 8)
				control_val = control[2]
				
				print "Control ID: "+repr(control_id)+", Value: "+repr(control_val)
			elif control[0] == 79:	#mode button?
				mode = control[1]
		
		#time.sleep(0.1)
	def start(self):
		try:
			while(True):
				self._poll()
		except KeyboardInterrupt:
			pass
			
if __name__== "__main__":
	n = NanoKontrol(3)
	n.start()
