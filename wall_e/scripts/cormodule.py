#! /usr/bin/env python
# -*- coding:utf-8 -*-


import numpy as np
import cv2


def crosshair(img, point, size, color):
    """ Desenha um crosshair centrado no point.
        point deve ser uma tupla (x,y)
        color é uma tupla R,G,B uint8
    """
    x,y = point
    cv2.line(img,(x - size,y),(x + size,y),color,2)
    cv2.line(img,(x,y - size),(x, y + size),color,2)


def coloca_texto(frame, org, texto):
    """
    cv2.putText(image, text, org, font, fontScale, color[, thickness[, 
                lineType[, bottomLeftOrigin]]])
    
    org = coordinates of the bottom-left corner of the text string in the image
    """
    font = cv2.FONT_HERSHEY_SIMPLEX
    cv2.putText(frame, texto, org, font, 1.0, (0, 255, 0), 1)


def center_of_contour(contorno):
    """ Retorna uma tupla (cx, cy) que desenha o centro do contorno"""
    M = cv2.moments(contorno)
    if M["m00"] > 0.001:
        # Usando a expressão do centróide definida em: https://en.wikipedia.org/wiki/Image_moment
        cX = int(M["m10"] / M["m00"])
        cY = int(M["m01"] / M["m00"])
        return (int(cX), int(cY))
    else:
        return (-1, -1)


# ENCONTRA CENTRO E ÁREA DO CONTORNO DE MAIOR AREA:
def acha_maior_contorno(gray):
    """ Estamos trabalhando com BGR como cores
        Retorna uma imagem com os contornos desenhados e a coordenada do centro do maior contorno
    """
    contornos = cv2.findContours(gray.copy(), cv2.RETR_TREE, cv2.CHAIN_APPROX_SIMPLE)[0]
    rgb = cv2.cvtColor(gray, cv2.COLOR_GRAY2RGB)
    cv2.drawContours(rgb, contornos, -1, [255, 0, 0], 1)
    
    p = (0,0)
    
    maior = None
    maior_area = 0
    for c in contornos:
        area = cv2.contourArea(c)
        if area > maior_area:
            maior_area = area
            maior = c

    if maior is not None:
        p = center_of_contour(maior)      
        cv2.drawContours(rgb, [maior], -1, [0, 0, 255], 2)
        crosshair(rgb, p, 5, (0,255,0))
    
    
    return p, maior_area, rgb


def identifica_pista(frame):

    # frame = cv2.flip(frame, -1) # flip 0: eixo x, 1: eixo y, -1: 2 eixos
    frame_hsv = cv2.cvtColor(frame, cv2.COLOR_BGR2HSV)

    cor_menor = np.array([22, 185, 115])
    cor_maior = np.array([34, 255, 255])
    segmentado_cor = cv2.inRange(frame_hsv, cor_menor, cor_maior)
    segmentado_cor = cv2.morphologyEx(segmentado_cor,cv2.MORPH_CLOSE,np.ones((7, 7)))

    centro_imagem = (frame.shape[1]//2, frame.shape[0]//2)

    centro_pista, area_pista, rgb = acha_maior_contorno(segmentado_cor)

    coloca_texto(rgb, (50, 30), 'Area pista: {:.2f}'.format(area_pista))

    cv2.imshow('pista', rgb)
    cv2.waitKey(1)

    return centro_pista, centro_imagem, area_pista


def identifica_creeper(frame, cor):

    # frame = cv2.flip(frame, -1) # flip 0: eixo x, 1: eixo y, -1: 2 eixos
    frame_hsv = cv2.cvtColor(frame, cv2.COLOR_BGR2HSV)

    if cor == 'blue':
        cor_menor = np.array([64, 38, 47])
        cor_maior = np.array([99, 172, 255])
    elif cor == 'orange':
        cor_menor = np.array([1, 183, 111])
        cor_maior = np.array([10, 255, 255])
    elif cor == 'green':
        cor_menor = np.array([35, 161, 69])
        cor_maior = np.array([77, 255, 255])

    segmentado_cor = cv2.inRange(frame_hsv, cor_menor, cor_maior)
    segmentado_cor = cv2.morphologyEx(segmentado_cor,cv2.MORPH_CLOSE,np.ones((7, 7)))

    centro_maior_contorno, area_maior_contorno, rgb = acha_maior_contorno(segmentado_cor)

    coloca_texto(rgb, (50, 30), 'Area creeper: {:.2f}'.format(area_maior_contorno))

    cv2.imshow('creeper', rgb)
    cv2.waitKey(1)

    return centro_maior_contorno, area_maior_contorno
