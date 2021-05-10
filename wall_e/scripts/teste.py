import rospy
from math import *


def retorna_pista(self, ponto):
        """
        Função para fazer o robô retornar a pista após pegar um creeper.
        Recebe o ponto para onde o robô deve apontar para retornar
        """

        print('Robô retornando para a pista! ')
        print('Parando o robô')
        self.pub.publish(self.vel_parado)
        rospy.sleep(2)
        
        x2, y2 = ponto

        # calcular theta
        theta = atan2(y2-self.y, x2-self.x)

        # ângulo que o robô deve virar para se ajustar com o ponto
        angulo = theta - self.alpha

        # girar theta - alpha para a esquerda
        tempo = angulo / self.w

        print('Corrigindo ângulo')
        # corrigir o ângulo
        if angulo < 0:
            self.pub.publish(self.vel_giro2)
        else:
            self.pub.publish(self.vel_giro)

        rospy.sleep(tempo)

        print('Andando para a pista')
        # andar 0.2 metros
        tempo2 = 0.2 / self.v
        self.pub.publish(self.vel_linear)
        rospy.sleep(tempo2)



"""
print('Creeper já foi pego, indo para a estação')
if self.area_net > 3000:
    estado = "segue estacao"

    if self.perto:
        estado = "parado"
        
else:
    if self.area_pista < 1000:
        estado = "meia volta"
    else:
        estado = "procura pista"
"""