Yale OpenHand Project - Software
========

Python libraries for easy control of the Dynamixel-based OpenHand designs. These Python objects depend on a modified version of the 'lib\_robotis.py' library from Georgia Tech. The library has been updated to properly control both MX and RX series servos, as well as accounting for possible header miscues as suggested by the pydynamixel library. Each hand object has pre-tested settings for servo bounds and torque output that we routinely use in our experiments. These values may vary for different hardware implementations or assembly processes. 

We also provide a set of scripts to help with hobby servo control via Pololu Maestro boards, specifically the Power HD 1501MG servo for select designs that can utilize them. The [Korg Nanokontrol board](http://www.korg.com/us/products/controllers/nanokontrol2/) is also supported as an affordable controller for both Dynamixel and hobby servos.

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

