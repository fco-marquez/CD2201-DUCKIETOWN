#!/usr/bin/env python2

import numpy as np
import os
import sys
import time
import rospy
from geometry_msgs.msg import Point


sys.path.insert(0, os.path.abspath('..'))

#from mlf.core.serial_control import SerialControl
from serial_control import SerialControl
from mk2robot import MK2Robot

robot = MK2Robot(link_lengths=[55, 39, 135, 147, 66.3])
robot_serial = SerialControl()
robot_serial.open_serial()

class template(object):
    def __init__(self,args):
        super(template,self).__init__()
        self.args = args
        self.sub = rospy.Subscriber("/goosebot/arm",Point, self.callback)

    def callback(self,msg):
        robot_serial.open_serial()

        X = msg.x
        Y = msg.y
        Z = msg.z
        print(X,Y,Z)
        q0, q1, q2 = robot.inverse_kinematics(X, Y, Z)
        robot_serial.write_servo(1, q0 + 45)
        robot_serial.write_servo(2, 90 - q1)
        robot_serial.write_servo(3, q2 + q1)
        time.sleep(1.2)

        robot_serial.read_status()
        robot_serial.read_sensors()
        robot_serial.run_effector()

        robot_serial.close_serial()

def main():
    rospy.init_node('test') #creacion y registro del nodo!

    obj = template('args') # Crea un objeto del tipo Template, cuya definicion se encuentra arriba

    #objeto.publicar() #llama al metodo publicar del objeto obj de tipo Template

    rospy.spin() #funcion de ROS que evita que el programa termine -  se debe usar en  Subscribers


if __name__ =='__main__':
    main()

