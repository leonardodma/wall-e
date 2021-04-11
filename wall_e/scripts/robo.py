#! /usr/bin/env python
# -*- coding:utf-8 -*-


# Ros imports
import rospy
from geometry_msgs.msg import Twist, Vector3
from sensor_msgs.msg import LaserScan, CompressedImage
from nav_msgs.msg import Odometry
from tf import transformations
import tf2_ros
from std_msgs.msg import Float64
import rospkg


# Open CV imports
from cv_bridge import CvBridge, CvBridgeError
import cv2

# Others imports
from math import *

from cormodule import *


class Robot():

    def __init__(self, v, w, cor_creeper):
        self.rospack = rospkg.RosPack() 
        self.cor_creeper = cor_creeper

        # Iniciando velocidades
        self.v = v
        self.w = w
        self.vel_ang = Twist(Vector3(0,0,self.v), Vector3(0,0,self.w))
        self.vel_linear = Twist(Vector3(v,0,0), Vector3(0,0,0))
        self.vel_direita = Twist(Vector3(v,0,0), Vector3(0,0,-self.w))
        self.vel_esquerda = Twist(Vector3(v,0,0), Vector3(0,0,self.w))
        self.vel_giro = Twist(Vector3(0,0,0), Vector3(0,0,self.w)) 
        self.vel_giro2 = Twist(Vector3(0,0,0), Vector3(0,0,-self.w)) 
        self.vel_parado = Twist(Vector3(0,0,0), Vector3(0,0,0)) 
        self.vel_lento = Twist(Vector3(0,0,(self.v)/2), Vector3(0,0,0)) 

        # Laser Scan
        self.perto = False  

        # Odometria
        self.x = 0
        self.y = 0
        self.z = 0 
        self.alpha = 0
        self.ponto = (0, 0)

        # Tópico de imagens
        self.bridge = CvBridge()
        self.cv_image = None
        topico_imagem = "/camera/image/compressed" # Use para robo virtual

        # Variáveis pista
        self.centro_pista = []
        self.centro_imagem = []
        self.area_pista = 0.0 # Variavel com a area do maior contorno
        
        # Variáveis Creeper
        self.centro_creeper = []
        self.area_creeper = 0.0 # Variavel com a area do maior contorno

        # Estado
        self.estado = 'segue pista'

        # Garra e Braço
        self.braco_frente = 0
        self.braco_levantado = 1.5
        self.braco_recolhido = -1.5
        self.garra_aberta = -1
        self.garra_fechada = 0
        self.tf_buffer = tf2_ros.Buffer()
        self.tfl = 0

        # Iniciando Tópicos
        self.pub = rospy.Publisher("cmd_vel", Twist, queue_size=3)
        self.recebe_scan = rospy.Subscriber("/scan", LaserScan, self.scaneou)
        self.image_sub = rospy.Subscriber(topico_imagem, CompressedImage, self.roda_todo_frame, queue_size=4, buff_size = 2**24)
        self.ref_odometria = rospy.Subscriber("/odom", Odometry, self.recebe_odometria)

        self.braco_publisher = rospy.Publisher('/joint1_position_controller/command', Float64, queue_size=1)
        self.garra_publisher = rospy.Publisher('/joint2_position_controller/command', Float64, queue_size=1)
        

    def scaneou(self, dado):
        """
        Função para receber os dados de proximidade do LaserScam
        """
        # print('Dado: {}'.format(dado.ranges[0]))
        if dado.ranges[0] <= 0.6:
            self.perto = True
        else:
            self.perto = False

        # print('Perto: {}'.format(self.perto))
    

    def recebe_odometria(self, data):
        """
        Função para receber os dados de posição da Odometria
        """
        self.x = data.pose.pose.position.x
        self.y = data.pose.pose.position.y

        quat = data.pose.pose.orientation
        lista = [quat.x, quat.y, quat.z, quat.w]
        angulos_rad = transformations.euler_from_quaternion(lista)

        self.alpha = angulos_rad[2] # mais facil se guardarmos alpha em radianos
    
    
    def roda_todo_frame(self, imagem):
        """
        Função para receber os dados de imagem da câmera do robô
        """
        try:
            cv_image = self.bridge.compressed_imgmsg_to_cv2(imagem, "bgr8")
            copia_imagem = cv_image.copy()
            # cv_image = cv2.flip(cv_image, -1) # Descomente se for robo real
            self.centro_pista, self.centro_imagem, self.area_pista =  identifica_pista(copia_imagem)
            self.centro_creeper, self.area_creeper = identifica_creeper(copia_imagem, self.cor_creeper)

            cv2.imshow("Camera", cv_image)
            cv2.waitKey(1)

        except CvBridgeError as e:
            print('ex', e)


    def segue_pista(self):
        """
        Função para seguir a pista
        """
        print(self.centro_pista)

        if len(self.centro_pista) != 0:
            if (self.centro_pista[0] > self.centro_imagem[0]):
                self.pub.publish(self.vel_direita)
            elif (self.centro_pista[0] < self.centro_imagem[0]):
                self.pub.publish(self.vel_esquerda)
        else:
            pass    

    def segue_creeper(self):
        """
        Função para seguir o creeper
        """
        if len(self.centro_creeper) != 0:
            if (self.centro_creeper[0] > self.centro_imagem[0]):
                self.pub.publish(self.vel_direita)
            elif (self.centro_creeper[0] < self.centro_imagem[0]):
                self.pub.publish(self.vel_esquerda)
        else:
            pass
    

    def meia_volta(self):
        """
        Função para dar meia volta ao encontrar um obstáculo
        """
        # Rotação
        angulo = pi
        tempo1 = angulo/self.w

        # Translação 
        espaco = 0.5
        tempo2 = espaco /self.v

        # Publishes velocidades
        self.pub.publish(self.vel_giro)
        rospy.sleep(tempo1)
        self.pub.publish(self.vel_linear)
        rospy.sleep(tempo2)
    

    def retorna_pista(self, ponto):
        """
        Função para fazer o robô retornar a pista após pegar um creeper.
        Recebe o ponto para onde o robô deve apontar para retornar
        """

        print('Robô retornando para a pista! ')
        self.pub.publish(self.vel_parado)
        rospy.sleep(2)
        
        x2, y2 = ponto

        # calcular theta
        theta = atan2(y2-self.y, x2-self.x)

        # ângulo que o robô deve virar para se ajustar com o ponto
        angulo = theta - self.alpha

        # girar theta - alpha para a esquerda
        tempo = angulo / self.w

        # corrigir o ângulo
        if angulo < 0:
            self.pub.publish(self.vel_giro2)
        else:
            self.pub.publish(self.vel_giro)

        rospy.sleep(tempo)

        # andar 0.5 metros
        tempo2 = 0.5 / self.v
        self.pub.publish(self.vel_linear)
        rospy.sleep(tempo2)
    

    def controla_garra(self):
        """
        Função para controlar o robô para agarrar o creeper quando ele aproxima do robô

        habilitar o controle da garra: roslaunch mybot_description mybot_control2.launch 
        """
        print('Controlando garra!')
        # Levantar o braço
        self.pub.publish(self.vel_parado)
        rospy.sleep(1)
        self.braco_publisher.publish(self.braco_frente)
        self.garra_publisher.publish(self.garra_aberta)
        rospy.sleep(1)

        # Aproxima do creeper lentamente
        tempo_aproxima = 0.5/((self.v)/2)
        self.pub.publish(self.vel_lento)
        rospy.sleep(tempo_aproxima)

        # Agarra o creeper
        self.garra_publisher.publish(self.garra_fechada)
        rospy.sleep(0.1)
        self.braco_publisher.publish(self.braco_levantado)
        rospy.sleep(0.1)

        # Retornar para a pista
        self.retorna_pista(self.ponto)
        print('Ponto para retornar: {}'.format(self.ponto))

    
    def determina_estado(self):
        """
        Função para determinar a ação do robo com base nos dados da pista, creeper,
        proximidade...
        """
        if self.area_creeper > 400:
            estado = "segue creeper"
            self.ponto = (self.x, self.y)
            if self.perto:
                estado = "controla garra"
                
        else:
            if self.area_pista < 900:
                estado = "procura pista"
            else:
                estado = "segue pista"
                if self.perto:
                    estado =  "meia volta"

        return estado


    def main(self):
        self.tfl = tf2_ros.TransformListener(self.tf_buffer) #conversao do sistema de coordenadas 

        self.estado = self.determina_estado()
        print('Estado: {}'.format(self.estado))
        
        if self.estado == "meia volta":
            self.meia_volta()
        elif self.estado == "segue creeper":
            self.segue_creeper()
        elif self.estado == "procura pista":
            self.pub.publish(self.vel_giro)
        elif self.estado == "segue pista":
            self.segue_pista()
        elif self.estado == "controla garra":
            self.controla_garra()
        
        #self.pub.publish(self.vel_parado)