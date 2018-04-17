#! /usr/bin/env python
# -*- coding:utf-8 -*-

__author__ = ["Rachel P. B. Moraes", "Igor Montagner", "Fabio Miranda"]


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
from sensor_msgs.msg import LaserScan
import identifica_features
#from sensor_msgs.msg import Imu

import cormodule

bridge = CvBridge()

cv_image = None

# Variáveis para permitir que o roda_todo_frame troque dados com a máquina de estados
media = []
centro = []
area = 0.0
media_feat = []
centro_feat = []
area_feat = 0.0
min_frente = 3.5
min_direita = 3.5
min_esquerda = 3.5
min_tras = 3.5


tolerancia_x = 50
tolerancia_y = 20
ang_speed = 0.2
area_ideal = 60000 # área da distancia ideal do contorno - note que varia com a resolução da câmera
tolerancia_area = 20000
fugindo = 0

# Atraso máximo permitido entre a imagem sair do Turbletbot3 e chegar no laptop do aluno
atraso = 1.5
check_delay = False # Só usar se os relógios ROS da Raspberry e do Linux desktop estiverem sincronizados




def roda_todo_frame(imagem):
	global cv_image
	global media
	global centro
	global area
	global media_feat
	global centro_feat
	global area_feat
	global GOODMATCH

	now = rospy.get_rostime()
	imgtime = imagem.header.stamp
	lag = now-imgtime
	delay = lag.secs
	if delay > atraso and check_delay==True:
		return 
	try:
		antes = time.clock()
		cv_image = bridge.compressed_imgmsg_to_cv2(imagem, "bgr8")
		media, centro, area = cormodule.identifica_cor(cv_image)
		media_feat, centro_feat, area_feat, GOODMATCH = identifica_features.identifica_feat(cv_image)
		depois = time.clock()
		cv2.imshow("Camera", cv_image)
	except CvBridgeError as e:
		print('ex', e)
	

def scaneou(dado):
	global min_frente
	global min_direita
	global min_esquerda
	global min_tras

	dados = np.array(dado.ranges).round(decimals=2)
	for i in range(len(np.array(dado.ranges).round(decimals=2))):
		if dados[i]<=0.05:
			dados[i]=3.5

	min_frente1 = min(dados[0:45])
	min_frente2 = min(dados[315:360])
	if(min_frente1>min_frente2):
		min_frente=min_frente1 
	else:
		min_frente=min_frente2
	min_direita = min(dados[225:315])
	min_esquerda = min(dados[45:135])
	min_tras = min(dados[135:225])






## Classes - estados
class Chegou(smach.State):
    def __init__(self):
        smach.State.__init__(self, outcomes=['parado','girando'])

    def execute(self, userdata):

		rospy.sleep(0.01)
		if media is None:
			return 'girando'
		if min_frente>0.4:
			return 'girando'
		if min_frente < 0.4:
			vel = Twist(Vector3(0, 0, 0), Vector3(0, 0, 0))
			velocidade_saida.publish(vel)
			return 'parado'

class Girando(smach.State):
    def __init__(self):
        smach.State.__init__(self, outcomes=['alinhou', 'girando','fugindofrente','fugindoesquerda','fugindodireita','fugindotras'])

    def execute(self, userdata):
		global velocidade_saida,fugindo

		rospy.sleep(0.01)

		if  min_tras <= 0.3:
			return 'fugindotras'
		if  min_frente <= 0.3:
			return 'fugindofrente'
		if  min_direita <= 0.3:
			return 'fugindodireita'
		if  min_esquerda <= 0.3:
			return 'fugindoesquerda'

		if media_feat is None or len(media_feat)==0:
			return 'girando'

		if  GOODMATCH>=40:
			fugindo=120
			vel = Twist(Vector3(1, 0, 0), Vector3(0, 0, 2))
			#vel = Twist(Vector3(0.05, 0, 0), Vector3(0, 0, 0.03))
			velocidade_saida.publish(vel)
			return 'girando'

		if fugindo >=0:
			fugindo-=1
			vel = Twist(Vector3(1, 0, 0), Vector3(0, 0, 2))
			#vel = Twist(Vector3(0.05, 0, 0), Vector3(0, 0, 0.03))
			velocidade_saida.publish(vel)
			return 'girando'

		

		if media is None or len(media)==0:
			return 'girando'

		if  math.fabs(media[0]) > math.fabs(centro[0] + tolerancia_x):
			vel = Twist(Vector3(0, 0, 0), Vector3(0, 0, -ang_speed))
			#vel = Twist(Vector3(0.05, 0, 0), Vector3(0, 0, 0.03))

			velocidade_saida.publish(vel)
			return 'girando'
		if math.fabs(media[0]) < math.fabs(centro[0] - tolerancia_x):
			vel = Twist(Vector3(0, 0, 0), Vector3(0, 0, ang_speed))
			#vel = Twist(Vector3(0.05, 0, 0), Vector3(0, 0, 0))

			velocidade_saida.publish(vel)
			return 'girando'
		else:
			vel = Twist(Vector3(0, 0, 0), Vector3(0, 0, 0))
			velocidade_saida.publish(vel)
			return 'alinhou'


class Centralizado(smach.State):
    def __init__(self):
        smach.State.__init__(self, outcomes=['alinhando', 'alinhado','parado'])

    def execute(self, userdata):
		global velocidade_saida
		rospy.sleep(0.01)


		if media is None:
			return 'alinhou'

		if min_frente<0.4:
			return	'parado'

		if  math.fabs(media[0]) > math.fabs(centro[0] + tolerancia_x):
			return 'alinhando'

		if math.fabs(media[0]) < math.fabs(centro[0] - tolerancia_x):
			return 'alinhando'

		else:
			vel = Twist(Vector3(0.2, 0, 0), Vector3(0, 0, 0))
			velocidade_saida.publish(vel)
			return 'alinhado'


class FugindoFrente(smach.State):
    def __init__(self):
        smach.State.__init__(self, outcomes=['fugindofrente', 'fugiu','fugindotras'])

    def execute(self, userdata):
		global velocidade_saida
		rospy.sleep(0.01)

		if media is None:
			return 'fugiu'

		if min_tras <= 0.25:
			return 'fugindotras'

		if  min_frente <= 0.4:
			vel = Twist(Vector3(-0.3, 0, 0), Vector3(0, 0, 0))
			velocidade_saida.publish(vel)
			return 'fugindofrente'
		else:
			vel = Twist(Vector3(0, 0, 0), Vector3(0, 0, 0))
			velocidade_saida.publish(vel)
			return 'fugiu'

class FugindoDireita(smach.State):
    def __init__(self):
        smach.State.__init__(self, outcomes=['fugindodireita', 'fugiu', 'fugindofrente'])

    def execute(self, userdata):
		global velocidade_saida

		rospy.sleep(0.01)
		if media is None:
			return 'fugiu'

		if  min_frente <= 0.4:
			return 'fugindofrente'

		if  min_direita <= 0.3:
			vel = Twist(Vector3(0, 0, 0), Vector3(0, 0, 0.3))
			velocidade_saida.publish(vel)
			return 'fugindodireita'

		else:
			vel = Twist(Vector3(0, 0, 0), Vector3(0, 0, 0))
			velocidade_saida.publish(vel)
			return 'fugiu'

class FugindoEsquerda(smach.State):
    def __init__(self):
        smach.State.__init__(self, outcomes=['fugindoesquerda', 'fugiu','fugindofrente'])

    def execute(self, userdata):
		global velocidade_saida
		rospy.sleep(0.01)

		if media is None:
			return 'fugiu'

		if  min_frente <= 0.4:
			return 'fugindofrente'


		if  min_esquerda <= 0.3:
			vel = Twist(Vector3(0, 0, 0), Vector3(0, 0, -0.3))
			velocidade_saida.publish(vel)
			return 'fugindoesquerda'

		else:
			vel = Twist(Vector3(0, 0, 0), Vector3(0, 0, 0))
			velocidade_saida.publish(vel)
			return 'fugiu'

class FugindoTras(smach.State):
    def __init__(self):
        smach.State.__init__(self, outcomes=['fugindotras', 'fugiu', ])

    def execute(self, userdata):
		global velocidade_saida
		rospy.sleep(0.01)

		if media is None:
			return 'fugiu'

		if  min_tras <= 0.3:
			vel = Twist(Vector3(0.05, 0, 0), Vector3(0, 0, 0))
			velocidade_saida.publish(vel)
			return 'fugindotras'

		else:
			vel = Twist(Vector3(0, 0, 0), Vector3(0, 0, 0))
			velocidade_saida.publish(vel)
			return 'fugiu'

# main
def main():
	global velocidade_saida
	global buffer
	rospy.init_node('cor_estados')



	# Para usar a webcam 

	#recebedor = rospy.Subscriber("/cv_camera/image_raw/compressed", CompressedImage, roda_todo_frame, queue_size=1, buff_size = 2**24)
	recebe_scan = rospy.Subscriber("/scan", LaserScan, scaneou, queue_size=4, buff_size = 2**24)
	#recebe_imu = rospy.Subscriber("/imu", Imu, leu_imu,queue_size = 1)
	recebedor = rospy.Subscriber("/raspicam_node/image/compressed", CompressedImage, roda_todo_frame, queue_size=4, buff_size = 2**24)
	velocidade_saida = rospy.Publisher("/cmd_vel", Twist, queue_size = 1)



	# Create a SMACH state machine
	sm = smach.StateMachine(outcomes=['terminei'])

	# Open the container
	with sm:
	    smach.StateMachine.add('GIRANDO', Girando(),
	                            transitions={'girando': 'GIRANDO',
	                            'alinhou':'CENTRO','fugindofrente':'FUGINDOFRENTE',
	                            'fugindodireita':'FUGINDODIREITA',
	                            'fugindoesquerda':'FUGINDOESQUERDA',
	                            'fugindotras':'FUGINDOTRAS'})
	    smach.StateMachine.add('CENTRO', Centralizado(),
	                            transitions={'alinhando': 'GIRANDO',
	                            'alinhado':'CENTRO','parado':'CHEGOU'})
	    smach.StateMachine.add('FUGINDOFRENTE', FugindoFrente(),
	                            transitions={'fugindofrente': 'FUGINDOFRENTE',
	                            'fugiu':'GIRANDO','fugindotras':'FUGINDOTRAS'})
	    smach.StateMachine.add('FUGINDODIREITA', FugindoDireita(),
	                            transitions={'fugindodireita': 'FUGINDODIREITA',
	                            'fugiu':'GIRANDO', 'fugindofrente':'FUGINDOFRENTE'})
	    smach.StateMachine.add('FUGINDOESQUERDA', FugindoEsquerda(),
	                            transitions={'fugindoesquerda': 'FUGINDOESQUERDA',
	                            'fugiu':'GIRANDO','fugindofrente':'FUGINDOFRENTE'})
	    smach.StateMachine.add('FUGINDOTRAS', FugindoTras(),
	                            transitions={'fugindotras': 'FUGINDOTRAS',
	                            'fugiu':'GIRANDO'})
	    smach.StateMachine.add('CHEGOU', Chegou(),
	                            transitions={'parado': 'CHEGOU',
	                            'girando':'GIRANDO'})
	     #smach.StateMachine.add('LONGE', Longe(), 
	    #                       transitions={'ainda_longe':'ANDANDO', 
	    #                                    'perto':'terminei'})
	    #smach.StateMachine.add('ANDANDO', Andando(), 
	    #                       transitions={'ainda_longe':'LONGE'})


	# Execute SMACH plan
	outcome = sm.execute()
	#rospy.spin()


if __name__ == '__main__':
    main()
