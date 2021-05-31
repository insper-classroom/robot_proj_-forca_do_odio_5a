#! /usr/bin/env python3
# -*- coding:utf-8 -*-

from __future__ import print_function, division

import rospy 

import numpy as np

import cv2

from geometry_msgs.msg import Twist, Vector3
from sensor_msgs.msg import LaserScan
from sensor_msgs.msg import Image, CompressedImage
from cv_bridge import CvBridge, CvBridgeError

from nav_msgs.msg import Odometry
from geometry_msgs.msg import Pose

from scipy.spatial.transform import Rotation as R

import math


low = np.array([22, 50, 50],dtype=np.uint8)
high = np.array([36, 255, 255],dtype=np.uint8)

def filter_color(bgr, low, high):
    """ REturns a mask within the range"""
    hsv = cv2.cvtColor(bgr, cv2.COLOR_BGR2HSV)
    mask = cv2.inRange(hsv, low, high)
    return mask     

def center_of_mass(mask):
    """ Retorna uma tupla (cx, cy) que desenha o centro do contorno"""
    M = cv2.moments(mask)
    m00 = max(M["m00"],1) # para evitar dar erro quando não há contornos
    cX = int(M["m10"] / m00)
    cY = int(M["m01"] / m00)
    return [int(cX), int(cY)]

def crosshair(img, point, size, color):
    """ Desenha um crosshair centrado no point.
        point deve ser uma tupla (x,y)
        color é uma tupla R,G,B uint8
    """
    x,y = point
    cv2.line(img,(x - size,y),(x + size,y),color,5)
    cv2.line(img,(x,y - size),(x, y + size),color,5)

def center_of_mass_region(mask, x1, y1, x2, y2):
    mask_bgr = cv2.cvtColor(mask, cv2.COLOR_GRAY2BGR)
    clipped = mask[y1:y2, x1:x2]
    c = center_of_mass(clipped)
    c[0]+=x1
    c[1]+=y1
    crosshair(mask_bgr, c, 10, (0,0,255))
    cv2.rectangle(mask_bgr, (x1, y1), (x2, y2), (255,0,0),2,cv2.LINE_AA)
    centro = (int(c[0]), int(c[1]))
    return mask_bgr, centro

font = cv2.FONT_HERSHEY_SIMPLEX

def texto(img, a, p, color = (0,50,100)):
    """Escreve na img RGB dada a string a na posição definida pela tupla p"""
    cv2.putText(img, str(a), p, font,1, color ,1,cv2.LINE_AA)
    

import statsmodels.api as sm


def ajuste_linear_x_fy(mask):
    """Recebe uma imagem já limiarizada e faz um ajuste linear
        retorna coeficientes linear e angular da reta
        e equação é da forma
        x = coef_angular*y + coef_linear
    """ 
    pontos = np.where(mask==255)
    ximg = pontos[1]
    yimg = pontos[0]

    if len(ximg) < 10: 
        return 0,0, [[0],[0]]

    yimg_c = sm.add_constant(yimg)
    model = sm.OLS(ximg,yimg_c)
    results = model.fit()
    coef_angular = results.params[1] 
    coef_linear =  results.params[0]
    return coef_angular, coef_linear, pontos


def ajuste_linear_grafico_x_fy(mask_in, print_eq = False): 
    """Faz um ajuste linear e devolve uma imagem rgb com aquele ajuste desenhado sobre uma imagem
       Trabalhando com x em funcão de y
    """

    scale_percent = 50 # percent of original size
    width = int(mask_in.shape[1] * scale_percent / 100)
    height = int(mask_in.shape[0] * scale_percent / 100)
    dim = (width, height)

    mask = cv2.resize(mask_in, dim)

    coef_angular, coef_linear, pontos  = ajuste_linear_x_fy(mask)
    if print_eq: 
        print("x = {:3f}*y + {:3f}".format(coef_angular, coef_linear))
    ximg = pontos[1]
    yimg = pontos[0]
    y_bounds = np.array([min(yimg), max(yimg)])
    x_bounds = coef_angular*y_bounds + coef_linear

    x_int = x_bounds.astype(dtype=np.int64)
    y_int = y_bounds.astype(dtype=np.int64)
    mask_bgr =  cv2.cvtColor(mask, cv2.COLOR_GRAY2BGR)    
    cv2.line(mask_bgr, (x_int[0], y_int[0]), (x_int[1], y_int[1]), color=(0,0,255), thickness=11);    
    return mask_bgr, coef_angular, coef_linear


import rospy
import numpy as np
import tf
import math
import cv2
import time
from geometry_msgs.msg import Twist, Vector3, Pose
from nav_msgs.msg import Odometry
from sensor_msgs.msg import Image, CompressedImage
from cv_bridge import CvBridge, CvBridgeError
import smach
import smach_ros


def identifica_cor(frame):
    '''
    Segmenta o maior objeto cuja cor é parecida com cor_h (HUE da cor, no espaço HSV).
    '''

    frame_hsv = cv2.cvtColor(frame, cv2.COLOR_BGR2HSV)

    cor_menor = np.array([100, 50, 50]) # Adaptado para azul
    cor_maior = np.array([120, 255, 255])
    segmentado_cor = cv2.inRange(frame_hsv, cor_menor, cor_maior)

    centro = (frame.shape[1]//2, frame.shape[0]//2)


    def cross(img_rgb, point, color, width,length):
        cv2.line(img_rgb, (int( point[0] - length/2 ), point[1] ),  (int( point[0] + length/2 ), point[1]), color ,width, length)
        cv2.line(img_rgb, (point[0], int(point[1] - length/2) ), (point[0], int( point[1] + length/2 ) ),color ,width, length) 


    segmentado_cor = cv2.morphologyEx(segmentado_cor,cv2.MORPH_CLOSE,np.ones((7, 7)))
	
    contornos, arvore = cv2.findContours(segmentado_cor.copy(), cv2.RETR_TREE, cv2.CHAIN_APPROX_SIMPLE) 

    maior_contorno = None
    maior_contorno_area = 0

    for cnt in contornos:
        area = cv2.contourArea(cnt)
        if area > maior_contorno_area:
            maior_contorno = cnt
            maior_contorno_area = area

    if not maior_contorno is None :
        cv2.drawContours(frame, [maior_contorno], -1, [0, 0, 255], 5)
        maior_contorno = np.reshape(maior_contorno, (maior_contorno.shape[0], 2))
        media = maior_contorno.mean(axis=0)
        media = media.astype(np.int32)
        cv2.circle(frame, (media[0], media[1]), 5, [0, 255, 0])
        cross(frame, centro, [255,0,0], 1, 17)
    else:
        media = (0, 0)

    font = cv2.FONT_HERSHEY_COMPLEX_SMALL
    cv2.putText(frame,"{:d} {:d}".format(*media),(20,100), 1, 4,(255,255,255),2,cv2.LINE_AA)
    cv2.putText(frame,"{:0.1f}".format(maior_contorno_area),(20,50), 1, 4,(255,255,255),2,cv2.LINE_AA)

    cv2.imshow('seg', frame)

    return media, centro, maior_contorno_area