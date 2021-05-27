#! /usr/bin/env python3
# -*- coding:utf-8 -*-

from __future__ import print_function, division
import rospy
import numpy as np
import numpy
import tf
import math
import cv2
from nav_msgs.msg import Odometry
from sensor_msgs.msg import Image, CompressedImage, LaserScan
from cv_bridge import CvBridge, CvBridgeError
from numpy import linalg
from tf import transformations
from tf import TransformerROS
import tf2_ros
from geometry_msgs.msg import Twist, Vector3, Pose, Vector3Stamped

from nav_msgs.msg import Odometry
from std_msgs.msg import Header
from aruco import identifica_aruco

print("EXECUTE ANTES da 1.a vez: ")
print("wget https://github.com/Insper/robot21.1/raw/main/projeto/ros_projeto/scripts/MobileNetSSD_deploy.caffemodel")
print("PARA TER OS PESOS DA REDE NEURAL")

import visao_module


bridge = CvBridge()

cv_image = None
media = []
centro = []
atraso = 1.5E9 # 1 segundo e meio. Em nanossegundos
identidade = []

distancia = 0
area = 0.0 # Variavel com a area do maior contorno

# Só usar se os relógios ROS da Raspberry e do Linux desktop estiverem sincronizados. 
# Descarta imagens que chegam atrasadas demais
check_delay = False 

resultados = [] # Criacao de uma variavel global para guardar os resultados vistos

x = 0
y = 0
z = 0
id = 0

frame = "camera_link"
# frame = "head_camera"  # DESCOMENTE para usar com webcam USB via roslaunch tag_tracking usbcam

tfl = 0

tf_buffer = tf2_ros.Buffer()
centrox = 0

def scaneou(dado):
    global distancia
    ranges = np.array(dado.ranges).round(decimals=2)
    distancia = ranges[0]

def segmenta_linha_amarela(bgr):
    """Não mude ou renomeie esta função
        deve receber uma imagem bgr e retornar os segmentos amarelos do centro da pista em branco.
        Utiliza a função cv2.morphologyEx() para limpar ruidos na imagem
    """
    hsv = cv2.cvtColor(bgr, cv2.COLOR_BGR2HSV)
    img = hsv.copy()
    
    cor1 = np.array([22, 50, 50], dtype=np.uint8)
    cor2 = np.array([35, 255, 255], dtype=np.uint8)
    mask = cv2.inRange(img, cor1, cor2)
    
    kernel = cv2.getStructuringElement(cv2.MORPH_ELLIPSE,(13,13))
    mask_1 = cv2.morphologyEx(mask, cv2.MORPH_OPEN, kernel )
    mask_final = cv2.morphologyEx(mask_1, cv2.MORPH_CLOSE, kernel )

    contornos, arvore = cv2.findContours(mask_final, cv2.RETR_TREE, cv2.CHAIN_APPROX_SIMPLE)

    centrox = []
    centroy = []


    for i in contornos:
        M = cv2.moments(i)
        # Usando a expressão do centróide definida em: https://en.wikipedia.org/wiki/Image_moment
        cX = int(M["m10"] / M["m00"])
        cY = int(M["m01"] / M["m00"])
        centro_x2 = int(cX)
        centro_y2 = int(cY)
        centrox.append(centro_x2)
        centroy.append(centro_y2)

    return mask_final, centrox[-1], centroy


# A função a seguir é chamada sempre que chega um novo frame
def roda_todo_frame(imagem):
    print("frame")
    global cv_image
    global media
    global centro
    global resultados
    global centrox
    global identidade

    now = rospy.get_rostime()
    imgtime = imagem.header.stamp
    lag = now-imgtime # calcula o lag
    delay = lag.nsecs
    # print("delay ", "{:.3f}".format(delay/1.0E9))
    if delay > atraso and check_delay==True:
        # Esta logica do delay so' precisa ser usada com robo real e rede wifi 
        # serve para descartar imagens antigas
        print("Descartando por causa do delay do frame:", delay)
        return 
    try:
        temp_image = bridge.compressed_imgmsg_to_cv2(imagem, "bgr8")
        # Note que os resultados já são guardados automaticamente na variável
        # chamada resultados
        centro, saida_net, resultados =  visao_module.processa(temp_image)        
        for r in resultados:
            # print(r) - print feito para documentar e entender
            # o resultado            
            pass
        
        # Desnecessário - Hough e MobileNet já abrem janelas
        cv_image = saida_net.copy()
        cv2.imshow("cv_image", cv_image)
        cv2.waitKey(1)
        imagem, centrox, centroy = segmenta_linha_amarela(cv_image)
        identidade = identifica_aruco(cv_image)
        cv2.imshow("centro", imagem)
        #print(imagem.shape) descobrindo o tamanho da tela
        print (centrox)
        

    except CvBridgeError as e:
        print('ex', e)
    
if __name__=="__main__":
    rospy.init_node("cor")

    topico_imagem = "/camera/image/compressed"

    recebedor = rospy.Subscriber(topico_imagem, CompressedImage, roda_todo_frame, queue_size=4, buff_size = 2**24)

    recebe_scan = rospy.Subscriber("/scan", LaserScan, scaneou)

    print("Usando ", topico_imagem)

    velocidade_saida = rospy.Publisher("/cmd_vel", Twist, queue_size = 1)

    tfl = tf2_ros.TransformListener(tf_buffer) #conversao do sistema de coordenadas 
    tolerancia = 25


    centro_img = 320
    margem_erro = 15
    parado = Twist(Vector3(0,0,0), Vector3(0,0,0))
    esquerda = Twist(Vector3(0,0,0), Vector3(0,0,0.15))
    direita = Twist(Vector3(0,0,0), Vector3(0,0,-0.15))
    frente = Twist(Vector3(0.15,0,0), Vector3(0,0,0))
    gira45 = Twist(Vector3(0,0,0), Vector3(0,0,35*math.pi/180))
    gira180 = Twist(Vector3(0,0,0), Vector3(0,0,180*math.pi/180))

    try:
        # Inicializando - por default gira no sentido anti-horário
        
        while not rospy.is_shutdown():
            #for r in resultados:
              #  print(r)
            for i in identidade:
                #print (i, distancia)
                if i == [100] and distancia < 1.5:
                    velocidade_saida.publish(frente)
                    rospy.sleep(2)
                    velocidade_saida.publish(parado)
                    rospy.sleep(1)
                    vel = gira45
                    velocidade_saida.publish(vel)
                    rospy.sleep(1)
            for i in identidade:
                #print (i, distancia)
                if i == [150] and distancia < 0.6:
                    velocidade_saida.publish(parado)
                    rospy.sleep(1)
                    vel = gira180
                    velocidade_saida.publish(vel)
                    rospy.sleep(1)
            if (centrox > centro_img + margem_erro):
                vel = direita
                print("direita")
            elif (centrox < centro_img - margem_erro):
                vel = esquerda
                print("esquerda")
            else:
                vel = frente
                print('frente')
            velocidade_saida.publish(vel)
            rospy.sleep(0.1)

    except rospy.ROSInterruptException:
        print("Ocorreu uma exceção com o rospy")
