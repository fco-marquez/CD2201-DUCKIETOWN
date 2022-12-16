#!/usr/bin/env python2

import numpy as np
import os
import sys
import time
import rospy
from geometry_msgs.msg import Point


class Template(object):
    def __init__(self,args):
        super(Template,self).__init__()
        self.args = args
        self.pub = rospy.Publisher("/goosebot/arm",Point, queue_size=10)
        

    def publicar(self,x,y,z):

        p = Point()
        p.x = x
        p.y = y
        p.z = z
        self.pub.publish(p)


def main():
    rospy.init_node('goosebot') #creacion y registro del nodo!

    obj = Template('args') # Crea un objeto del tipo Template, cuya definicion se encuentra arriba

    #rospy.spin() #funcion de ROS que evita que el programa termine -  se debe usar en  Subscribers

    for i in range(1,10):
        ix = 100+i*10
        iy = 100+i*10
        iz = 100+i*10
        obj.publicar(ix,iy,iz) #llama al metodo publicar del objeto obj de tipo Template
        time.sleep(2)


if __name__ =='__main__':
    main()


