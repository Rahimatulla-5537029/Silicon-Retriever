#!/usr/bin/env python3
# license removed for brevity

import rospy as rp
from geometry_msgs.msg import Pose, Vector3
from mirte_msgs.msg import *
from mirte_msgs.srv import *
from sensor_msgs.msg import *
from std_srvs.srv import *
import time

SetRightSpeed = rp.ServiceProxy('/mirte/set_right_speed', SetMotorSpeed)
SetLeftSpeed = rp.ServiceProxy('/mirte/set_left_speed', SetMotorSpeed)
SetLeftServo=rp.ServiceProxy('/mirte/set_left_servo_angle',SetServoAngle)
SetRightServo=rp.ServiceProxy('/mirte/set_right_servo_angle',SetServoAngle)


SetLeftServo(0)        # Arms positioned backwards
SetRightServo(180)     
time.sleep(1)


SetLeftServo(180-0)   # Arms positioned forwards
SetRightServo(0)
time.sleep(1)
    
SetLeftSpeed(50)     # Robot drives towards ball
SetRightSpeed(50)
    
time.sleep(1)

SetLeftServo(180-90) # Robots picks up ball and rotates halfway through
SetRightServo(90)

time.sleep(1)

SetLeftSpeed(0)     # Robot stops driving
SetRightSpeed(0)

time.sleep(4)        # Arms return to backwards position
SetLeftServo(180-0)
SetRightServo(0)



    
def nodestart():
    rp.init_node('wb2232_pythontestnode', anonymous=True)
    rp.Timer(rp.Duration(0.5),distancecallback)
    print('ok, wb2232_pythontestnode is gestart, je kunt hem stoppen met CTRL-c. De motoren stoppen dan niet, dat moet je nog zelf op de command line doen.')
    rp.spin()

if __name__ == '__main__':
    nodestart()
