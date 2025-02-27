#!/usr/bin/env python
# -*- coding: utf-8 -*-
#
# robot_randomsearch.py
# Contact (ce fichier uniquement): nicolas.bredeche(at)upmc.fr
# 
# Description:
#   Template pour robotique evolutionniste simple 
#   Ce code utilise pySpriteWorld, développé par Yann Chevaleyre (U. Paris 13)
# 
# Dépendances:
#   Python 3.x
#   Matplotlib
#   Pygame
# 
# Historique: 
#   2016-03-28__23:23 - template pour 3i025 (IA&RO, UPMC, licence info)
#   2018-03-28__14:06 - refactoring complet, synchronisation avec les autres exemples, implémentation fonctionnelle du random search
#   2019-04-02__11:42 - passage Python 3.x
#
# Cet exemple illustre la recherche aléatoire de contrôleur, dont la performance est mesurée en fonction d'une tâche fixée par l'utilisateur.
# Le contrôleur est ici un réseau de neurones de type Perceptron, c'est à dire que les vitesses de translation et rotation sont fixées par une combinaison linéaire
# entre entrées sensorielles et paramètres, en appliquant une fonction tangente hyperbolique pour obtenir une réponse non-linéaire et bornée entre -1 et +1.
# Les paramètres contenu dans le génome sont ici limités à un vecteur donc chaque élément prend soit -1 (inhibition), 0 (annulation), +1 (excitation).
# Il s'agit d'un problème d'optimisation de type boîte noir, ou la performance d'une solution donne peu d'information sur comment modifier les paramètres de cette
# solution pour l'améliorer.
# Ce code donne les bases pour implémenter des algorithmes optimisation de type évolution artificielle (algorithmes génétiques, stratégies d'évolution, etc.)
# A noter que ce code peut aussi facilement être étendu pour optimiser le comportement de groupe de robots (plutôt qu'un seul robot)
#
# Aide sur le code
#   - La méthode "stepController" de la classe Agent, qui définit comment est utilisé le génome pour moduler les actions du robot en fonction des informations
# sensorielles
#   - Les méthodes "updateFitness" de la classe Agent, qui permettent de définir une métrique pour la tâche visée
#   - le contenu du main (tout en bas), ou est défini l'algorithme d'exploration (ici: un random search), y compris l'initialisation, la modification et le
# stockage des génomes, et leur évaluation
# Et aussi, éventuellement:
#   - Partie "variables globales"
#   - La fonction setupAgents (permet de placer les robots au début de la simulation)
#   - La fonction setupArena (permet de placer des obstacles au début de la simulation)
#   - il n'est pas conseillé de modifier les autres parties du code.
# 
# Aide sur la partie optimisation:
#   - pour modifier un genome, il faut modifier sa taille (dans le main lors de l'initialisation) et son utilisation (dans le stepController)
#   - pour définir l'objectif, il faut écrire une fonction fitness. Ce que mesure la fonction fitness peut être plus ou moins directement reliée à l'objectif
# (p.ex.: si l'objectif est d'optimiser des explorateurs, la fonction fitness peut être une mesure de la capacité à se déplacer en ligne droite en évitant les murs)
#   - pour obtenir un tirage issu d'une distribution normale, il faut utiliser la fonction gauss. 
# Exemple: random.gauss(0,1) <=> N(0,1) (i.e. tirage d'une distribution normale centrée sur 0 et d'écart type 1)

from robosim import *
from random import *
import math
import time
import sys
import atexit
from itertools import count
import matplotlib.pyplot as plt
import numpy as np

from copy import deepcopy


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
screen_width=512 #512,768,... -- multiples de 32  
screen_height=512 #512,768,... -- multiples de 32
nbAgents = 10

arena = 4

maxSensorDistance = 30              # utilisé localement.
maxRotationSpeed = 5
maxTranslationSpeed = 3
SensorBelt = [-170,-80,-40,-20,+20,40,80,+170]  # angles en degres des senseurs

showSensors = True
frameskip = 200   # 0: no-skip. >1: skip n-1 frames
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
    name = "Team Grenouille" # A modifier avec le nom de votre équipe

    translationValue = 0 # ne pas modifier directement
    rotationValue = 0 # ne pas modifier directement

    params = []
    fitness = 0
    previousPos = (0,0)

    pos = set()
    
    def __init__(self,robot):
        self.id = Agent.agentIdCounter
        Agent.agentIdCounter = Agent.agentIdCounter + 1
        #print "robot #", self.id, " -- init"
        self.robot = robot

    def getRobot(self):
        return self.robot

    # =-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=
    # =-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=
    # =-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=

    def evaluate(self,_params):
        
        self.resetPosition()
        self.resetFitness()
        self.params = list(_params)

        for i in range(maxIterations):
            updateSensors()
            #others.step()
            self.step()
            #self.updateFitness1() # pour maximiser la distance au centre de l'arène
            #self.updateFitness2() # pour maximiser la distance parcourue a chaque pas de temps
            self.updateFitness4() # pour maximiser la distance parcourue a chaque pas de temps, en pénalisant les commandes de rotation
            game.mainiteration()

        return self.fitness

    def resetPosition(self):
        p = self.robot
        p.set_position( screen_width/2+random()*4 , screen_height/2+random()*4 )
        p.oriente( random()*360 ) # DEBUG
        
    # =-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=
    # =-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=
    # =-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=

    def resetFitness(self):
        prevPos = self.robot.get_centroid()
        self.fitness = 0

    def updateFitness1(self):
        currentPos = self.robot.get_centroid()
        self.fitness += math.sqrt(abs(currentPos[0]**2-(screen_width/2)**2)) + math.sqrt(abs(currentPos[1]**2-(screen_height/2)**2)) # somme de la distance au centre de l'arene a chaque pas de temps
    
    def updateFitness2(self):
        currentPos = self.robot.get_centroid()
        self.fitness += math.sqrt(abs(currentPos[0]**2-self.previousPos[0]**2)) + math.sqrt(abs(currentPos[1]**2-self.previousPos[1]**2)) # a chaque pas de temps, ajoute la distance parcourue depuis t-1
        self.previousPos = currentPos

    def updateFitness3(self):
        currentPos = self.robot.get_centroid()
        self.fitness += ( 1 - abs(self.rotationValue/maxRotationSpeed) ) * math.sqrt(abs(currentPos[0]**2-self.previousPos[0]**2)) + math.sqrt(abs(currentPos[1]**2-self.previousPos[1]**2)) # a chaque pas de temps, ajoute la distance parcourue depuis t-1, avec une pénalité si rotation
        self.previousPos = currentPos

    def updateFitness4(self):
        currentPos = self.robot.get_centroid()
        currentPos = tuple(map(lambda x: int(x), currentPos))
        if currentPos not in self.pos:
            self.fitness += 5
            self.pos.add(currentPos)
        else:
            self.fitness -= 2
        self.previousPos = currentPos

    def updateFitness5(self): # anti parasite + découverte
        currentPos = self.robot.get_centroid()
        currentPos = tuple(map(lambda x: int(x), currentPos))
        for i in range(len(SensorBelt)):
            if self.getDistanceAtSensor(i)==2 and self.getRobotInfoAtSensor(0)['teamname'] != self.teamname :
                self.fitness -= 2
            else:
                self.fitness += 8
        if currentPos not in self.pos:
            self.fitness += 5
            self.pos.add(currentPos)
        else:
            self.fitness -= 2
        self.previousPos = currentPos


    # =-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=
    # =-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=
    # =-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=

    def stepController(self, ennemy=False):
        rotation =0
        translation = 0
        #if self.id == 0 :    
        translation = 0
        rotation = 0
        '''
        sensorMinus80 = self.getDistanceAtSensor(1)
        sensorMinus40 = self.getDistanceAtSensor(2)
        sensorMinus20 = self.getDistanceAtSensor(3)
        sensorPlus20 = self.getDistanceAtSensor(4)
        sensorPlus40 = self.getDistanceAtSensor(5)
        sensorPlus80 = self.getDistanceAtSensor(6)
        '''
        if not ennemy:
            '''
            if len(self.params) != 14: # vérifie que le nombre de paramètres donné est correct
                print ("[ERROR] number of parameters is incorrect. Exiting.")
                exit()
            # Perceptron: a linear combination of sensory inputs with weights (=parameters). Use an additional parameters as a bias, and apply hyperbolic tangeant to ensure result is in [-1,+1]
            translation =  math.tanh( sensorMinus80 * self.params[0] + sensorMinus40 * self.params[1] + sensorMinus20 * self.params[2] + sensorPlus20 * self.params[3] + sensorPlus40 * self.params[4] + sensorPlus80 * self.params[5] + self.params[6]) 
            rotation =  math.tanh( sensorMinus80 * self.params[7] + sensorMinus40 * self.params[8] + sensorMinus20 * self.params[9] + sensorPlus20 * self.params[10] + sensorPlus40 * self.params[11] + sensorPlus80 * self.params[12] + self.params[13] )

            #print ("robot #", self.id, "[r =",rotation," - t =",translation,"]")
            '''

            sensorMinus170 = self.getDistanceAtSensor(0)
            sensorMinus80 = self.getDistanceAtSensor(1)
            sensorMinus40 = self.getDistanceAtSensor(2)
            sensorMinus20 = self.getDistanceAtSensor(3)
            sensorPlus20 = self.getDistanceAtSensor(4)
            sensorPlus40 = self.getDistanceAtSensor(5)
            sensorPlus80 = self.getDistanceAtSensor(6)
            sensorPlus170 = self.getDistanceAtSensor(7)

            translation =  math.tanh( sensorMinus80 * self.params[0] + sensorMinus40 * self.params[1] + sensorMinus20 * self.params[2] + sensorPlus20 * self.params[3] + sensorPlus40 * self.params[4] + sensorPlus80 * self.params[5] + sensorMinus170 * self.params[6] + self.params[7]) 
            rotation =  math.tanh( sensorMinus80 * self.params[8] + sensorMinus40 * self.params[9] + sensorMinus20 * self.params[10] + sensorPlus20 * self.params[11] + sensorPlus40 * self.params[12] + sensorPlus80 * self.params[13] + sensorPlus170 * self.params[14] + self.params[15])
                    
            self.setRotationValue(rotation)
            self.setTranslationValue(translation)
        else :
            if self.getObjectTypeAtSensor(3) == 1 or self.getObjectTypeAtSensor(4) == 1 or self.getObjectTypeAtSensor(1) == 1 or self.getObjectTypeAtSensor(0) == 1 :
                n = randint(1,4)
                if n == 4 :
                    translation = 1
                    rotation = 1
            else :
                # Evite les murs et les robots
                for i in range(len(SensorBelt)):
                    rotation += self.getObjectTypeAtSensor(i)

        self.setRotationValue( rotation )
        self.setTranslationValue( translation )

        return

    # =-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=
    # =-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=
    # =-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=

    def step(self, ennemy=False):
        self.stepController(ennemy)
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
            #print ("[WARNING] translation value not in [-1,+1]. Normalizing.")
            value = maxTranslationSpeed
        elif value < -1:
            #print ("[WARNING] translation value not in [-1,+1]. Normalizing.")
            value = -maxTranslationSpeed
        else:
            value = value * maxTranslationSpeed
        self.translationValue = value

    def setRotationValue(self,value):
        if value > 1:
            #print ("[WARNING] translation value not in [-1,+1]. Normalizing.")
            value = maxRotationSpeed
        elif value < -1:
            #print ("[WARNING] translation value not in [-1,+1]. Normalizing.")
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


def setupArena():
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


def setupArena4():
    for i in range(3,11):
        addObstacle(row=3,col=i)
    for i in range(3,9):
        addObstacle(row=i,col=11)
    for i in range(8,12):
        addObstacle(row=9,col=i)
    for i in range(5,12):
        addObstacle(row=11,col=i)
    for i in range(5,12):
        addObstacle(row=i,col=5)
    for i in range(12,14):
        addObstacle(row=i,col=11)
    for i in range(6,10):
        addObstacle(row=5,col=i)   


def updateSensors():
    global sensors 
    # throw_rays...(...) : appel couteux (une fois par itération du simulateur). permet de mettre à jour le masque de collision pour tous les robots.
    sensors = throw_rays_for_many_players(game,game.layers['joueur'],SensorBelt,max_radius = maxSensorDistance+game.player.diametre_robot() , show_rays=showSensors)
    

def stepWorld():
    updateSensors()

    # chaque agent se met à jour. L'ordre de mise à jour change à chaque fois (permet d'éviter des effets d'ordre).
    shuffledIndexes = [i for i in range(len(agents))]
    shuffle(shuffledIndexes)
    agents[0].step()
    shuffledIndexes.remove(0)
    for i in range(len(agents)-1):
        agents[shuffledIndexes[i]].step(ennemy=True)
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

def mute(individu,pMute):
    nouvelIndividu = []
    for e in individu:
        if random() < pMute:
            nouvelIndividu.append( (e + 1) % 2 )
        else:
            nouvelIndividu.append( e )
    return nouvelIndividu


''''''''''''''''''''''''''''''
''''''''''''''''''''''''''''''
''''''''''''''''''''''''''''''
''''''''''''''''''''''''''''''
'''  Main loop             '''
''''''''''''''''''''''''''''''
''''''''''''''''''''''''''''''
''''''''''''''''''''''''''''''
''''''''''''''''''''''''''''''

''''''''''''''''''''''''''''''''''''
''' Initialisation du monde      '''
''''''''''''''''''''''''''''''''''''

init('empty',MyTurtle,screen_width,screen_height) # display is re-dimensioned, turtle acts as a template to create new players/robots
game.auto_refresh = False # display will be updated only if game.mainiteration() is called
game.frameskip = frameskip
atexit.register(onExit)

setupArena()


setupAgents()
game.mainiteration()

''''''''''''''''''''''''''''''''''''
'''  Apprentissage/optimisation  '''
''''''''''''''''''''''''''''''''''''

'''
for evaluationIt in range(maxEvaluations):
    stepWorld()
    game.mainiteration()
'''

print ("Optimizing.")

game.frameskip = 200 # affichage à vitesse (très) rapide -- Benchmark (2018/3/28): macbook pro 3.1Ghz 12" core i7 'early 2015': 1250 updates/sec 

bestFitness = 0 # init with worst value
bestParams = []
bestEvalIt = 0

maxEvaluations = 100 # budget en terme de nombre de robots évalués au total
maxIterations = 750 # temps passé pour évaluer _un_ robot
nbReevaluations = 4
genomeSize = 16
taillePopulation = 10
mutation = 1 / genomeSize

def mute(fitnessAMuter,mutation):
    nombreMutations = mutation * genomeSize
    fitnessTemp = deepcopy(fitnessAMuter)
    for i in range(int(nombreMutations)):
        temp = randint(0,genomeSize-1)
        while(fitnessTemp[temp] == fitnessAMuter[temp]) :
            fitnessTemp[temp] = uniform(-1.0,1.0)
    return fitnessTemp

# Une mutation c'est bon, mais plusieurs mutations : forcer à muter sur différents gènes

# Initialisation
# Parent

for i in range(genomeSize):  # taille du genome 
    bestParams.append(randint(-1,+1)) # construit un genome composé de N valeurs -1, 0 ou +1

# Enfants

paramEnfant = deepcopy(bestParams)

params = []

for i in range(taillePopulation):
    params.append(paramEnfant)

'''
x = np.arange(maxEvaluations)
y = np.arange(maxEvaluations)
z = np.arange(maxEvaluations)
'''

for evaluationIt in range(maxEvaluations): # générations
    updateSensors()
    new_param = []

    fitness = 0
    fitness_best = 0
    best_enfant = []
    #print ("Evaluation #"), evaluationIt

    # genere un nouveau jeu de paramètres
    # evalue les parametres

    for enfant in params:
        enfant = mute(enfant,mutation)
        new_param.append(enfant)


    for enfant in params:
        '''
        for i in range(1, len(agents)):
            agents[i].step(True)
        '''
        fitness = agents[0].evaluate(enfant)
        if fitness > fitness_best:
            fitness_best = fitness
            best_enfant = enfant
    '''
    for i in range (nbReevaluations): # N évaluations indépendantes
        fitness += agents[0].evaluate(params)
    '''

    params = new_param

    if bestFitness < fitness_best:
        bestParams = list(best_enfant)
        bestFitness = fitness_best
        bestEvalIt = evaluationIt

    #stepWorld()
    print ("Evaluation =", evaluationIt)
    print ("\tParameters:", str(bestParams))
    print ("\tFitness:", fitness_best, "(best:", bestFitness,")")

print("bestParams =",bestParams)

'''
    #print(str(evaluationIt)+","+str(fitness)+","+str(bestFitness))

    #print(str(bestFitness))

    x[evaluationIt] = evaluationIt
    y[evaluationIt] = fitness
    #z[evaluationIt] = bestFitness


plt.plot(x,y,color='red')
plt.show()
'''

game.frameskip = 1 # affichage à vitesse normal

print ("Display best individual")
print ("\tParameters:", str(bestParams))
i = 0
while i<1:
    print ("\tTest #",i)
    i = i + 1

    # evalue les parametres
    fitness = agents[0].evaluate(bestParams)

    print ("\t\tFitness:", fitness, "(original recorded fitness:", bestFitness,", measured at evaluation",bestEvalIt,")")
    print ("\t\tGenome:", bestParams)