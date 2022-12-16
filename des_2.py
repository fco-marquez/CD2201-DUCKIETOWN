#!/usr/bin/env python

import numpy as np
import rospy #importar ros para python
from std_msgs.msg import String, Int32, Float32 # importar mensajes de ROS tipo String y tipo Int32
from geometry_msgs.msg import Twist,Point # importar mensajes de ROS tipo geometry / Twist
from sensor_msgs.msg import Joy
from duckietown_msgs.msg import Twist2DStamped

class Template(object):
    def __init__(self, args):
        super(Template, self).__init__()
        self.args = args
        rospy.Subscriber("duckiebot/joy", Joy, self.callback)   
        self.pub= rospy.Publisher("/duckiebot/wheels_driver_node/car_cmd", Twist2DStamped, queue_size=10)
        self.twist= Twist2DStamped()
        self.controller = rospy.Subscriber("duckiebot/detecciones2", Point, self.point) 
	rospy.Subscriber("duckiebot/joy", Joy, self.callback)
	self.Dr= 910000

    def point(self,point):
        self.Dr = point.x   
        print(self.Dr)

	if self.b==1:
            y= 0

        if abs(self.a[0])<=0.1:
            self.a[0]=0
            
        if self.Dr < 6 and self.a[0] > 0:
            y=0    
        
        omega_1= mapeo(self.a[1], -1,1,-20,20)
        
        self.twist.v = y
        self.twist.omega= -omega_1
       
    
    def callback(self,msg):
	self.a=list(msg.axes)
	self.b=msg.buttons[1]
	if self.b==1:
		y = 0
		w = 0

	else:
		y = mapeo(self.a[1], -2,2,-20,20)
		w = -mapeo(self.a[0], -1,1,-20,20)
		
		if abs(self.a[0])<=0.1:
		    w = 0
		if abs(self.a[1])<=0.1:
		    y= 0

	
	print(self.twist)
	self.twist.v = y
	self.twist.omega = w
        self.pub.publish(self.twist)

def mapeo(valor,vi,vf,nvi,nvf):
    return np.interp(valor,[vi,vf],[nvi,nvf])

    #def listener(self,msg):
        #rospy.init_node('listener', anonymous=True)
        
        

def main():
    rospy.init_node('template') #creacion y registro del nodo!

    obj = Template('args') # Crea un objeto del tipo Template, cuya definicion se encuentra arriba

    #objeto.publicar() #llama al metodo publicar del objeto obj de tipo Template

    rospy.spin() #funcion de ROS que evita que el programa termine -  se debe usar en  Subscribers

if __name__ =='__main__':
    main()
