#! /usr/bin/env python
# -*- coding:utf-8 -*-

import rospy

import numpy as np

from geometry_msgs.msg import Twist, Vector3
from sensor_msgs.msg import LaserScan


def scaneou(dado):
	print("Faixa valida: ", dado.range_min , " - ", dado.range_max )
	print("Leituras:")
	grau = 0
	dados = np.array(dado.ranges).round(decimals=2)
	for i in range(len(np.array(dado.ranges).round(decimals=2))):
		if dados[i]<=0.1:
			dados[i]=3.5

	print(grau, dados[i])
	frente=0
	direita=0
	tras=0
	esquerda=0

	# for i in range(45):
	# 	frente	+= dados[315+i] + dados[i]
	# 	esquerda	+= dados[45+i] 	+ dados[i+90]
	# 	tras	+= dados[135+i] + dados[i+180]
	# 	direita+= dados[180+i] + dados[i+225]

	min_frente1 = min(dados[0:45])
	min_frente2 = min(dados[315:360])
	if(min_frente1>min_frente2):
		min_frente=min_frente1 
	else:
		min_frente=min_frente2
	min_direita = min(dados[225:315])
	min_esquerda = min(dados[45:135])
	min_tras = min(dados[135:225])

	# frente	=frente/90
	# direita	=direita/90
	# esquerda=esquerda/90
	# tras	=tras/90
	#frente,tras,esquerda,direita = (frente/90,tras/90,esquerda/90,direita/90)

	# print("frente : {0}".format(min_frente))
	# print("tras : {0}".format(min_tras))
	# print("esquerda : {0}".format(min_esquerda))
	# print("direita : {0}".format(min_direita))
	# print()

	return (min_frente,min_tras,min_esquerda,min_direita)
	#print(np.array(dado.ranges).round(decimals=2))
	#print(len(np.array(dado.ranges).round(decimals=2)))
	#print("Intensities")
	#print(np.array(dado.intensities).round(decimals=2))


	


if __name__=="__main__":

	#rospy.init_node("le_scan")

	velocidade_saida = rospy.Publisher("/cmd_vel", Twist, queue_size = 3 )
	recebe_scan = rospy.Subscriber("/scan", LaserScan, scaneou)

	
	while not rospy.is_shutdown():
		#velocidade = Twist(Vector3(0, 0, 0), Vector3(0, 0, 0))
		#velocidade_saida.publish(velocidade)
		rospy.sleep(2)
