#!/usr/bin/env python2
# -*- coding: utf-8 -*-
import rospy
import time
from sensor_msgs.msg import LaserScan
from ackermann_msgs.msg import AckermannDriveStamped
from nav_msgs.msg import Odometry
from geometry_msgs.msg import Point
from geometry_msgs.msg import Quaternion
import math
import os

orientacao_objetivo = Quaternion(x=0,y=0,z=0,w=0)
orientacao_atual = Quaternion(x=0,y=0,z=0,w=0)
posicao_objetivo = Point(x = 5, y = 5)
posicao_atual = Point(0,0,0);

posicao = 2

os.chdir(r'/catkin_ws/src/novo_pacote_teste_lego_team/node')

def callback(msg):
	global orientacao_atual
	global posicao_atual
	orientacao_atual = msg.pose.pose.orientation
	posicao_atual = msg.pose.pose.position

#essa funcao abre um arquivo .csv e retorna um vetor com os dados presentes na linha passada como parametro
def ler_csv(n_linha):
	arquivo = open(r'pontos.csv', "r") #coloca o arquivo de pontos dentro do path do node, ou declara um path absoluto
			              #tipo "/home/luizfpastuch/Documents/trajectory_map/pontos.csv" pra deixar em uma pasta diferente (acho que precisa)
	linhas = arquivo.readlines() #le todas as linhas do arquivo
	linha = linhas[n_linha-1] #pega a linha que quer
	vetor_string = linha.split(',') #divide a linha a partir das virgulas (achei que seria mais dificil kk)
	vetor_float = [] #declara o novo vetor pra retornar
	for i in range(len(vetor_string)): #converte cada item do vetor de string pra float
		vetor_float.append(float(vetor_string[i]))
	arquivo.close() #fecha o arquivo
	return vetor_float[0:2] #retorna o novo vetor

class FollowTheGap(object):    

    def __init__(self):	
	
	#rospy.loginfo(str(objetivo.x))
	rate = rospy.Rate(40)
	direc_msg = AckermannDriveStamped()
        while not rospy.is_shutdown():

		global orientacao_objetivo
		global orientacao_atual
		global posicao_objetivo

		angulo_atual = 2*math.asin(orientacao_atual.z)			
		
		# Essas condições servem para que os angulos fiquem entre 0<theta<180 e 0>theta>-180

		if(posicao_atual.x < posicao_objetivo.x):
			angulo_objetivo = math.atan((posicao_objetivo.y - posicao_atual.y)/(posicao_objetivo.x - posicao_atual.x))				
		else:	
			if (posicao_atual.y < posicao_objetivo.y):
				angulo_objetivo = math.pi+math.atan((posicao_objetivo.y - posicao_atual.y)/(posicao_objetivo.x - posicao_atual.x))	
			else:
				angulo_objetivo = -math.pi+math.atan((posicao_objetivo.y - posicao_atual.y)/(posicao_objetivo.x - posicao_atual.x))	
		
		#rospy.loginfo(str(angulo_objetivo*180/math.pi))
		erro = angulo_objetivo -angulo_atual

		Kerro = -1	
		K = 1

		distancia_objetivo = math.sqrt(math.pow((posicao_objetivo.y - posicao_atual.y),2) + math.pow((posicao_objetivo.x - posicao_atual.x),2))

		#velocidade = Kerro*abs(erro)+K*distancia_objetivo/27
		global posicao
		velocidade = 4
		
		if(distancia_objetivo < 0.5):			
			posicao = posicao + 1

		novo_objetivo = ler_csv(posicao)
		posicao_objetivo = Point(x = novo_objetivo[0], y = novo_objetivo[1])
						
		#if(distancia_objetivo < 0.5):
		#	if(posicao == 1):
		#		posicao_objetivo = Point(x = 15, y = 9)
		#		posicao = 2
		#	elif(posicao == 2):
		#		posicao_objetivo = Point(x = 18, y = 5)
		#		posicao = 3
		#	elif(posicao == 3):
		#		posicao_objetivo = Point(x = 13, y = 0)
		#		posicao = 1

		direc_msg.drive.speed = velocidade
		direc_msg.drive.steering_angle = erro

		gap_pub.publish(direc_msg)
		rate.sleep()


def main():
	rospy.init_node('new_lego_team_node', anonymous = False)

	#self.sub = rospy.Subscriber('/scan', LaserScan, callback)
	sub = rospy.Subscriber('/odom', Odometry, callback)

	fg = FollowTheGap()
	rospy.spin()


gap_pub = rospy.Publisher('/drive', AckermannDriveStamped, queue_size=10)

main()
