openhand
========

Python libraries for easy control of the Dynamixel-based OpenHand designs. These Python objects depend on a modified version of the 'lib\_robotis.py' library from Georgia Tech. The library has been updated to properly control both MX and RX series servos, as well as accounting for possible header miscues as suggested by the pydynamixel library.

We assume that all OpenHand designs (for now) will use exclusively Dynamixel servos, so each hand should just be a collection of servos with certain desired operating bounds.

[OpenHand Website](http://www.eng.yale.edu/grablab/openhand/)

========
Basic Usage:

python -i openhand.py

>> T = Model\_T([port name], [main servo id])
>> T.close([desired torque for closing grasp (0.0-1.0)])
>> T.moveMotor([index of servo], [desired position (0.0-1.0)])
>> T.release()

>> T = Model\_T42([port name], [servo #1 id], [servo #2 id], [Dynamixel series ("RX", "MX")])
>> T.close([desired tendon length for close (0.0-1.0)])
>> T.release()

=========
Example Usage:

python -i openhand.py

>> T = Model\_T("/dev/ttyUSB0",1)
>> T.close(0.3)
>> T.release()

>> T = Model\_T42("/dev/ttyUSB0",6,7,"MX")
>> T.close(0.6)
>> T.release()
>> T.moveMotor(0,0.6)
>> T.moveMotor(1,0.6)

