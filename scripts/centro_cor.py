#! /usr/bin/env python3
# -*- coding:utf-8 -*-

__author__ = ["Rachel P. B. Moraes", "Igor Montagner", "Fabio Miranda"]


import rospy
import numpy as np
import tf
import math
import cv2
from geometry_msgs.msg import Twist, Vector3, Pose
from nav_msgs.msg import Odometry
from sensor_msgs.msg import Image, CompressedImage
from cv_bridge import CvBridge, CvBridgeError
import cores


bridge = CvBridge()

cv_image = None
media = []
centro = []
atraso = 1.5E9
maior_area = 0

area = 0.0 

check_delay = False 

def roda_todo_frame(imagem):
	print("frame")
	global cv_image
	global media
	global centro
	global maior_area

	now = rospy.get_rostime()
	imgtime = imagem.header.stamp
	lag = now-imgtime
	delay = lag.nsecs
	print("delay ", "{:.3f}".format(delay/1.0E9))
	if delay > atraso and check_delay==True:
		print("Descartando por causa do delay do frame:", delay)
	try:
		antes = rospy.Time.now()
		cv_image = bridge.compressed_imgmsg_to_cv2(imagem, "bgr8")
		media, centro, maior_area =  cores.identifica_cor_verde(cv_image)
		depois = rospy.Time.now()
		cv2.imshow("Camera", cv_image)
	except CvBridgeError as e:
		print('ex', e)
	
if __name__=="__main__":
	rospy.init_node("cor")

	topico_imagem = "camera/image/compressed"

	recebedor = rospy.Subscriber(topico_imagem, CompressedImage, roda_todo_frame, queue_size=4, buff_size = 2**24)
	print("Usando ", topico_imagem)
	
	velocidade_saida = rospy.Publisher("/cmd_vel", Twist, queue_size = 1)

	try:

		while not rospy.is_shutdown():
			vel = Twist(Vector3(0,0,0), Vector3(0,0,0))
			if len(media) != 0 and len(centro) != 0:
				print("Média dos vermelhos: {0}, {1}".format(media[0], media[1]))
				print("Centro dos vermelhos: {0}, {1}".format(centro[0], centro[1]))

				if (media[0] > centro[0]):
					vel = Twist(Vector3(0.0,0,0), Vector3(0,0,-0.1))
				if (media[0] < centro[0]):
					vel = Twist(Vector3(0.0,0,0), Vector3(0,0,0.1))
			velocidade_saida.publish(vel)
			rospy.sleep(0.1)
			print(maior_area) # Area = 2200

	except rospy.ROSInterruptException:
	    print("Ocorreu uma exceção com o rospy")