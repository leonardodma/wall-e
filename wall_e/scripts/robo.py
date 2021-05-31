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
from aruco import *
from visao_module import *


class Robot():

    def __init__(self, v, w, cor_creeper, id_creeper, estacao):
        self.rospack = rospkg.RosPack() 
        self.cor_creeper = cor_creeper
        self.id_creeper = id_creeper
        self.estacao = estacao

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
        self.vel_lento = Twist(Vector3(0,0,(self.v)/3), Vector3(0,0,0)) 

        # Laser Scan
        self.perto = False  

        # Odometria
        self.x = 0
        self.y = 0
        self.z = 0 
        self.alpha = 0

        # Tópico de imagens
        self.bridge = CvBridge()
        self.cv_image = None
        topico_imagem = "/camera/image/compressed" # Use para robo virtual

        # Variáveis pista
        self.centro_pista = []
        self.centro_imagem = []
        self.area_pista = 500 # Variavel com a area do maior contorno
        
        # Variáveis Creeper
        self.centro_creeper = []
        self.area_creeper = 0.0 # Variavel com a area do maior contorno

        # Variaveis Mobile Net
        self.centro_net = []
        self.area_net = []

        # Estado
        self.estado = 'segue pista'
        self.creeper_pego = False
        self.giro_efetuado = False
        self.seguir_aruco = False
        self.id_correto = False

        # Garra e Braço
        self.braco_frente = 0
        self.braco_levantado = 1.5
        self.braco_recolhido = -1.5
        self.garra_aberta = -1
        self.garra_fechada = 0
        self.tf_buffer = tf2_ros.Buffer()
        self.tfl = 0

        # Aruco 
        self.ids = None
        self.match_id = 0
        self.match_100 = 0
        self.centro_aruco = None

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
        if dado.ranges[0] <= 0.3:
            self.perto = True
        else:
            self.perto = False


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
            aruco_image = copia_imagem
            # saida_net, resultados =  processa(copia_imagem)  

            # for r in resultados:
            #     if r[0] == self.estacao and r[1] > 30:
            #         x1, y1 = r[2]
            #         x2, y2 = r[3]
            #         self.centro_net = (int((x2-x1)/2), int((y2-y1)/2))
            #         self.area_net = (x2-x1)*(y2-y1)
            #         print(self.centro_net)
            #         print(self.area_net)
            #     else:
            #         self.area_net = 0.0

            self.centro_pista, self.centro_imagem, self.area_pista =  identifica_pista(copia_imagem)
            self.centro_creeper, self.area_creeper = identifica_creeper(copia_imagem, self.cor_creeper)

            c, self.ids = identifica_id(aruco_image)

            if self.ids == 100:
                self.centro_aruco = c

            cv2.circle(aruco_image, self.centro_aruco, radius=2, color=(0, 0, 255), thickness=5)

            cv2.imshow("Aruco", aruco_image)
            cv2.waitKey(1)

        except CvBridgeError as e:
            print('ex', e)


    def segue_pista(self):
        """
        Função para seguir a pista
        """

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
    

    def segue_aruco(self):
        """
        Função para seguir aruco
        """
        if self.centro_aruco != (0, 0):
            if (self.centro_aruco[0] > self.centro_imagem[0]):
                self.pub.publish(self.vel_direita)
            elif (self.centro_aruco[0] < self.centro_imagem[0]):
                self.pub.publish(self.vel_esquerda)

    
    def segue_estacao(self):
        """
        Função para seguir a estação
        """
        if len(self.centro_net) != 0:
            if (self.centro_net[0] > self.centro_imagem[0]):
                self.pub.publish(self.vel_direita)
            elif (self.centro_net[0] < self.centro_imagem[0]):
                self.pub.publish(self.vel_esquerda)
        else:
            pass
    

    def controla_garra(self):
        """
        Função para controlar o robô para agarrar o creeper quando ele aproxima do robô

        habilitar o controle da garra: roslaunch mybot_description mybot_control2.launch 
        """
        print('Levantando braço!')
        # Levantar o braço
        self.pub.publish(self.vel_parado)
        rospy.sleep(1)
        self.braco_publisher.publish(self.braco_frente)
        self.garra_publisher.publish(self.garra_aberta)
        rospy.sleep(1)


        print('Agarrando o creeper')
        # Agarra o creeper
        self.garra_publisher.publish(self.garra_fechada)
        rospy.sleep(0.1)
        self.braco_publisher.publish(self.braco_levantado)
        rospy.sleep(0.1)
        

    def interpreta_id_curva(self):
        if self.ids == 100:
            self.match_100 += 1

        print('Matchs qrcode: {}'.format(self.match_100))

        if self.match_100 > 10 and self.match_100 < 24 :
            self.seguir_aruco = True
        else:
            self.seguir_aruco = False

    
    def interpreta_id_creeper(self):

        if self.ids == self.id_creeper:
            self.match_id += 1

        print('Matchs qrcode: {}'.format(self.match_100))

        if not self.creeper_pego:
            if self.match_id > 20:
                self.id_correto = True


    def determina_estado(self):
        """
        Função para determinar a ação do robo com base nos dados da pista, creeper,
        proximidade...
        """
        self.interpreta_id_curva()


        # Creeper ainda não foi pego
        if not self.creeper_pego:
            if self.area_creeper > 500:
                self.interpreta_id_creeper()
                estado = "segue creeper"
                    
                if self.perto:
                    if self.id_correto:
                        # Creeper foi pego
                        self.creeper_pego = True
                        estado = "controla garra"
                    else:
                        estado = "procura pista"

            else:
                if self.seguir_aruco:
                    estado = "segue aruco"
                
                else:
                    if self.area_pista < 1000:
                        estado = "procura pista"
                    else:
                        estado = "segue pista"
        else:
            if self.seguir_aruco:
                    estado = "segue aruco"
                
            else:
                if self.area_pista < 1000:
                    estado = "procura pista"
                else:
                    estado = "segue pista"

        return estado


    def main(self):
        self.tfl = tf2_ros.TransformListener(self.tf_buffer) #conversao do sistema de coordenadas 

        self.estado = self.determina_estado()
        print('Estado: {}'.format(self.estado))
        
        
        if self.estado == "segue creeper":
            self.segue_creeper()
        elif self.estado == "procura pista":
            self.pub.publish(Twist(Vector3(0,0,0), Vector3(0,0,(2*self.w))))
        elif self.estado == "segue pista":
            self.segue_pista()
        elif self.estado == "controla garra":
            self.controla_garra()
        # elif self.estado == "segue estacao":
        #     self.segue_estacao()
        elif self.estado == "segue aruco":
            self.segue_aruco()
        
        
        rospy.sleep(0.1)