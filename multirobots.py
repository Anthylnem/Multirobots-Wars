#!/usr/bin/env python
# -*- coding: utf-8 -*-
#
# multirobot.py
# Contact (ce fichier uniquement): nicolas.bredeche(at)upmc.fr
# 
# Description:
#   Template pour simulation mono- et multi-robots type khepera/e-puck/thymio
#   Ce code utilise pySpriteWorld, développé par Yann Chevaleyre (U. Paris 13)
#
# Dépendances:
#   Python 3.x
#   Matplotlib
#   Pygame
# 
# Historique: 
#   2016-03-28__23h23 - template pour 3i025 (IA&RO, UPMC, licence info)
#   2018-03-27__20h55 - réécriture de la fonction step(.), utilisation de valeurs normalisées (senseurs et effecteurs). Interface senseurs, protection du nombre de commandes de translation et de rotation (1 commande par appel de step(.) pour chacune)
# 	2018-03-28__10:00 - renommage de la fonction step(.) en stepController(.), refactoring
#   2019-04-02__11:42 - passage Python 3.x
#
# Aide: code utile
#   - Partie "variables globales": en particulier pour définir le nombre d'agents. La liste SensorBelt donne aussi les orientations des différentes capteurs de proximité.
#   - La méthode "stepController" de la classe Agent
#   - La fonction setupAgents (permet de placer les robots au début de la simulation)
#   - La fonction setupArena (permet de placer des obstacles au début de la simulation)
#   - il n'est pas conseillé de modifier les autres parties du code.
# 

from robosim import *
from random import random, shuffle, randint
import time
import math
import sys
import atexit   


'''''''''''''''''''''''''''''
'''''''''''''''''''''''''''''
'''  Aide                 '''
'''''''''''''''''''''''''''''
'''''''''''''''''''''''''''''

#game.setMaxTranslationSpeed(3) # entre -3 et 3
# size of arena: 
#   screenw,screenh = taille_terrain()
#   OU: screen_width,screen_height

'''''''''''''''''''''''''''''
'''''''''''''''''''''''''''''
'''  variables globales   '''
'''''''''''''''''''''''''''''
'''''''''''''''''''''''''''''

game = Game()

agents = []

arena = 0

screen_width=512 #512,768,... -- multiples de 32  
screen_height=512 #512,768,... -- multiples de 32
nbAgents = 1

maxSensorDistance = 30              # utilisé localement.
maxRotationSpeed = 5
maxTranslationSpeed = 1

SensorBelt = [-170,-80,-40,-20,+20,40,80,+170]  # angles en degres des senseurs (ordre clockwise)

maxIterations = -1 # infinite: -1

showSensors = True
frameskip = 0   # 0: no-skip. >1: skip n-1 frames
verbose = True

'''''''''''''''''''''''''''''
'''''''''''''''''''''''''''''
'''  Classe Agent/Robot   '''
'''''''''''''''''''''''''''''
'''''''''''''''''''''''''''''

class Agent(object):
    
    agentIdCounter = 0 # use as static
    id = -1
    robot = -1
    name = "Equipe Alpha" # A modifier avec le nom de votre équipe

    translationValue = 0 # ne pas modifier directement
    rotationValue = 0 # ne pas modifier directement

    def __init__(self,robot):
        self.id = Agent.agentIdCounter
        Agent.agentIdCounter = Agent.agentIdCounter + 1
        #print ("robot #", self.id, " -- init")
        self.robot = robot

    def getRobot(self):
        return self.robot

    # =-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=
    # =-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=
    # =-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=

    def stepController(self):

        #print ("robot #", self.id, " -- step")

        p = self.robot

        # actions
        # valeur de paramètre entre -1 et +1.
        # cette valeur sera converti ensuite entre:
        #  - pour setTranslation: entre -maxTranslationSpeed et +maxTranslationSpeed
        #  - pour setRotation: entre -maxRotationSpeed et +maxRotationSpeed
        # Attention:
        #   ces fonctions *programment* la commande motrice, mais *ne l'exécute pas*
        #   la dernière valeur allouée exécutée. Chaque fonction doit donc être appelé une seule fois.
        
        translation = 1
        rotation = 0
        stratégie = 4

        
        #params = [0, 1, 0, 1, -1, 1, 1, -1, -1, -1, 1, 1, 1, 0]
        #params = [1, 0, 1, 1, 0, 1, 0, -1, -1, 0, 1, 1, 1, -1]
        params = [0, -1, -1, 0.8449572470497386, 0.2772933724660105, -1, 0.9121020462155531, 0.45633640795837005, 0, 0.7507189530663856, 0, -0.6309759587498023, -0.09976779273277026, 0]

        sensorMinus170 = self.getDistanceAtSensor(0)
        sensorMinus80 = self.getDistanceAtSensor(1)
        sensorMinus40 = self.getDistanceAtSensor(2)
        sensorMinus20 = self.getDistanceAtSensor(3)
        sensorPlus20 = self.getDistanceAtSensor(4)
        sensorPlus40 = self.getDistanceAtSensor(5)
        sensorPlus80 = self.getDistanceAtSensor(6)
        sensorPlus170 = self.getDistanceAtSensor(7)

        translation =  math.tanh( sensorMinus80 * params[0] + sensorMinus40 * params[1] + sensorMinus20 * params[2] + sensorPlus20 * params[3] + sensorPlus40 * params[4] + sensorPlus80 * params[5] + sensorMinus170 * params[6] + params[7]) 
        rotation =  math.tanh( sensorMinus80 * params[8] + sensorMinus40 * params[9] + sensorMinus20 * params[10] + sensorPlus20 * params[11] + sensorPlus40 * params[12] + sensorPlus80 * params[13] + sensorPlus170 * params[14] + params[15])
                
        self.setRotationValue(rotation)
        self.setTranslationValue(translation)

        # monitoring - affiche diverses informations sur l'agent et ce qu'il voit.
        # pour ne pas surcharger l'affichage, je ne fais ca que pour le player 1
        if verbose == True and self.id == 0:

            efface()    # j'efface le cercle bleu de l'image d'avant
            color( (0,0,255) )
            circle( *game.player.get_centroid() , r = 22) # je dessine un rond bleu autour de ce robot
	
			# monitoring (optionnel - changer la valeur de verbose)
            print ("Robot #"+str(self.id)+" :")
            for i in range(len(SensorBelt)):
                print ("\tSenseur #"+str(i)+" (angle: "+ str(SensorBelt[i])+"°)")
                print ("\t\tDistance  :",self.getDistanceAtSensor(i))
                print ("\t\tType      :",self.getObjectTypeAtSensor(i)) # 0: rien, 1: mur ou bord, 2: robot
                print ("\t\tRobot info:",self.getRobotInfoAtSensor(i)) # dict("id","centroid(x,y)","orientation") (si pas de robot: renvoi "None" et affiche un avertissement dans la console
                print(rotation)
                print(translation)

        return

    # =-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=
    # =-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=
    # =-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=


    def step(self):
        self.stepController()
        self.move()

    def move(self):
        self.robot.forward(self.translationValue)
        self.robot.rotate(self.rotationValue)

    def getDistanceAtSensor(self,id):
        sensor_infos = sensors[self.robot] # sensor_infos est une liste de namedtuple (un par capteur).
        return min(sensor_infos[id].dist_from_border,maxSensorDistance) / maxSensorDistance

    def getObjectTypeAtSensor(self,id):
        if sensors[self.robot][id].dist_from_border > maxSensorDistance:
            return 0 # nothing
        elif sensors[self.robot][id].layer == 'joueur':
            return 2 # robot
        else:
            return 1 # wall/border

    def getRobotInfoAtSensor(self,id):
        if sensors[self.robot][id].dist_from_border < maxSensorDistance and sensors[self.robot][id].layer == 'joueur':
            otherRobot = sensors[self.robot][id].sprite
            info = {'id': otherRobot.numero, 'centroid': otherRobot.get_centroid(), 'orientation': otherRobot.orientation()}
            return info
        else:
            #print ("[WARNING] getPlayerInfoAtSensor(.): not a robot!")
            return None

    def setTranslationValue(self,value):
        if value > 1:
            print ("[WARNING] translation value not in [-1,+1]. Normalizing.")
            value = maxTranslationSpeed
        elif value < -1:
            print ("[WARNING] translation value not in [-1,+1]. Normalizing.")
            value = -maxTranslationSpeed
        else:
            value = value * maxTranslationSpeed
        self.translationValue = value

    def setRotationValue(self,value):
        if value > 1:
            print ("[WARNING] translation value not in [-1,+1]. Normalizing.")
            value = maxRotationSpeed
        elif value < -1:
            print ("[WARNING] translation value not in [-1,+1]. Normalizing.")
            value = -maxRotationSpeed
        else:
            value = value * maxRotationSpeed
        self.rotationValue = value


'''''''''''''''''''''''''''''
'''''''''''''''''''''''''''''
'''  Fonctions init/step  '''
'''''''''''''''''''''''''''''
'''''''''''''''''''''''''''''

def setupAgents():
    global screen_width, screen_height, nbAgents, agents, game

    # Make agents
    nbAgentsCreated = 0
    for i in range(nbAgents):
        while True:
            p = -1
            while p == -1: # p renvoi -1 s'il n'est pas possible de placer le robot ici (obstacle)
                p = game.add_players( (random()*screen_width , random()*screen_height) , None , tiled=False)
            if p:
                p.oriente( random()*360 )
                p.numero = nbAgentsCreated
                nbAgentsCreated = nbAgentsCreated + 1
                agents.append(Agent(p))
                break
    game.mainiteration()


def setupArena0():
    for i in range(6,13):
        addObstacle(row=3,col=i)
    for i in range(3,10):
        addObstacle(row=12,col=i)
    addObstacle(row=4,col=12)
    addObstacle(row=5,col=12)
    addObstacle(row=6,col=12)
    addObstacle(row=11,col=3)
    addObstacle(row=10,col=3)
    addObstacle(row=9,col=3)

def setupArena3():
    '''for i in range(0,7):
        addObstacle(row=i,col=8)

    for i in range(8,16):
        addObstacle(row=i,col=8)'''

def updateSensors():
    global sensors
    # throw_rays...(...) : appel couteux (une fois par itération du simulateur). permet de mettre à jour le masque de collision pour tous les robots.
    sensors = throw_rays_for_many_players(game,game.layers['joueur'],SensorBelt,max_radius = maxSensorDistance+game.player.diametre_robot() , show_rays=showSensors)

def stepWorld():
    global sensors
    
    updateSensors()

    # chaque agent se met à jour. L'ordre de mise à jour change à chaque fois (permet d'éviter des effets d'ordre).
    shuffledIndexes = [i for i in range(len(agents))]
    shuffle(shuffledIndexes)
    for i in range(len(agents)):    
        agents[shuffledIndexes[i]].step()
    return


'''''''''''''''''''''''''''''
'''''''''''''''''''''''''''''
'''  Fonctions internes   '''
'''''''''''''''''''''''''''''
'''''''''''''''''''''''''''''

def addObstacle(row,col):
    # le sprite situe colone 13, ligne 0 sur le spritesheet
    game.add_new_sprite('obstacle',tileid=(0,13),xy=(col,row),tiled=True)

class MyTurtle(Turtle): # also: limit robot speed through this derived class
    maxRotationSpeed = maxRotationSpeed # 10, 10000, etc.
    def rotate(self,a):
        mx = MyTurtle.maxRotationSpeed
        Turtle.rotate(self, max(-mx,min(a,mx)))

def onExit():
    print ("\n[Terminated]")

'''''''''''''''''''''''''''''
'''''''''''''''''''''''''''''
'''  Main loop            '''
'''''''''''''''''''''''''''''
'''''''''''''''''''''''''''''

init('empty',MyTurtle,screen_width,screen_height) # display is re-dimensioned, turtle acts as a template to create new players/robots
game.auto_refresh = False # display will be updated only if game.mainiteration() is called
game.frameskip = frameskip
atexit.register(onExit)

if arena == 0:
    setupArena0()
elif arena == 3:
    setupArena3()

setupAgents()
game.mainiteration()

iteration = 0
while iteration != maxIterations:
    stepWorld()
    game.mainiteration()
    iteration = iteration + 1
