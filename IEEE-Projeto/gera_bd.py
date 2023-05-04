#!/usr/bin/env python
import rospy
from mediapipe_holistic_ros.msg import MediaPipeHolistic
import numpy as np
from tkinter import *
import time
import cv2
import csv
import math

global pararGravar
global leftHandHuman, rightHandHuman
global start_time
global imagem_raw
global leftHandHumanAng
global classe
global lefthandhumanarray,righthandhumanarray,boca_listarray,rosto_listarray,bracodireito_listarray,bracoesquerdo_listarray
global repeticao
global rows, rowslefthand, rowsrighthand,rowsbocalist,rowsrostolist,rowsbracodireitolist,rowsbracoesquerdolist
global ehLeftHand,ehRightHand,ehBoca,ehRosto,EhBracoDireito,EhBracoEsquerdo

ehLeftHand = False
ehRightHand = False
ehBoca = False
ehRosto = False
EhBracoDireito = False
EhBracoEsquerdo = False
classe = ""
lefthandhumanarray = []
righthandhumanarray = []
boca_listarray = []
rosto_listarray = []
bracodireito_listarray = []
bracoesquerdo_listarray = []

rows = []
rowslefthand = []
rowsrighthand = []
rowsbocalist = []
rowsrostolist = []
rowsbracodireitolist = []
rowsbracoesquerdolist = []
repeticao = 0
pararGravar = True

global leftHandHuman, rightHandHuman, face_mesh_list, pose_list
global pontoReferenciaSeg1, pontoReferenciaSeg2, pontoReferenciaSeg3, pontoReferenciaSeg4, pontoReferenciaSeg5
global ArrayAllPontos, ArrayPontosSeg1, ArrayPontosSeg2, ArrayPontosSeg3, ArrayPontosSeg4,ArrayPontosSeg5,ArrayPontosSeg6,ArrayPontosSeg7
global msg

global start_time
global imagem_raw
global leftHandHumanOld, rightHandHumanOld, face_mesh_list_old, pose_list_old
global leftHandHumanFim, leftHandHumanInit,rightHandHumanFim, rightHandHumanInit,face_mesh_list_Fim,face_mesh_list_Init,pose_list_Init,pose_list_Fim

leftHandHumanInit = []
leftHandHumanFim = []
rightHandHumanInit= []
rightHandHumanFim= []
face_mesh_list_Fim = []
face_mesh_list_Init = []
leftHandHumanOld = []
rightHandHumanOld = []
leftHandHuman = []
rightHandHuman = []
face_mesh_list_old = []
face_mesh_list = []
pose_list_old = []
pose_list = []
ArrayAllPontos = []
ArrayPontosSeg1 = []
ArrayPontosSeg2 = []
ArrayPontosSeg3 = []
ArrayPontosSeg4 = []

pontoReferenciaSeg1 = 0
pontoReferenciaSeg2 = 0
pontoReferenciaSeg3 = 0
pontoReferenciaSeg4 = 168
pontoReferenciaSeg5 = 168
imagem_raw = ""


def calculaMaior():
    global  pararGravar,ArrayAllPontos, ArrayPontosSeg1
    global leftHandHumanFim, leftHandHumanInit,rightHandHumanFim, rightHandHumanInit,face_mesh_list_Fim,face_mesh_list_Init
    global rows,rowslefthand,rowsrighthand,rowsbocalist,rowsrostolist,rowsbracodireitolist,rowsbracoesquerdolist
    global ehLeftHand,ehRightHand,ehBoca,ehRosto,EhBracoDireito,EhBracoEsquerdo
    pararGravar = True
    SomaDPSeg1 = 0
    SomaDPSeg2 = 0
    SomaDPSeg3 = 0
    SomaDPSeg4 = 0

    #print(len(ArrayAllPontos))
    print("________Calculando Seguimentos...______________")
    
    #____________Seguimento 1___________________________
    print ("_________Seguimento 1_____________")
    #cria variaveis globais
    if ArrayPontosSeg1:
        for x in range (0, len(ArrayPontosSeg1[0][0])):
                globals()['arraySomaSeg1%s' % x] = []
        #atribui a
        for i in range(0,len(ArrayPontosSeg1)): 
            for x in range (0, len(ArrayPontosSeg1[0][0])):
                globals()['arraySomaSeg1%s' % x].append(ArrayPontosSeg1[i][0][x])    
        #printa as variaveis:
        for x in range (0, len(ArrayPontosSeg1[0][0])):
            SomaDPSeg1 = SomaDPSeg1 + np.std(globals()['arraySomaSeg1%s' % x], ddof = 1)
            print(np.std(globals()['arraySomaSeg1%s' % x], ddof = 1))
        SomaDPSeg1  = SomaDPSeg1/(len(ArrayPontosSeg1[0][0]))
    else:
        SomaDPSeg1 = 0
    #___________________________________________________
        #____________Seguimento 2___________________________
    print ("_________Seguimento 2_____________")
    #cria variaveis globais
    if ArrayPontosSeg2:
        for x in range (0, len(ArrayPontosSeg2[0][0])):
                globals()['arraySomaSeg2%s' % x] = []
        #atribui a
        for i in range(0,len(ArrayPontosSeg2)): 
            for x in range (0, len(ArrayPontosSeg2[0][0])):
                globals()['arraySomaSeg2%s' % x].append(ArrayPontosSeg2[i][0][x])    
        #printa as variaveis:
        for x in range (0, len(ArrayPontosSeg2[0][0])):
            SomaDPSeg2 = SomaDPSeg2 + np.std(globals()['arraySomaSeg2%s' % x], ddof = 1)
            print(np.std(globals()['arraySomaSeg2%s' % x], ddof = 1))
        SomaDPSeg2  = SomaDPSeg2/(len(ArrayPontosSeg2[0][0]))
    else:
        SomaDPSeg2 = 0
    # #___________________________________________________
    #     #____________Seguimento 3___________________________
    print ("_________Seguimento 3_____________")
    #cria variaveis globais
    if ArrayPontosSeg3:
        for x in range (0, len(ArrayPontosSeg3[0][0])):
                globals()['arraySomaSeg3%s' % x] = []
        #atribui a
        for i in range(0,len(ArrayPontosSeg3)): 
            for x in range (0, len(ArrayPontosSeg3[0][0])):
                globals()['arraySomaSeg3%s' % x].append(ArrayPontosSeg3[i][0][x])    
        #printa as variaveis:
        for x in range (0, len(ArrayPontosSeg3[0][0])):
            SomaDPSeg3 = SomaDPSeg3 + np.std(globals()['arraySomaSeg3%s' % x], ddof = 1)
            print(np.std(globals()['arraySomaSeg3%s' % x], ddof = 1))
        SomaDPSeg3  = SomaDPSeg3/(len(ArrayPontosSeg3[0][0]))
    else:
        SomaDPSeg3 = 0
    # #___________________________________________________
        #     #____________Seguimento 4___________________________
    print ("_________Seguimento 4_____________")
    #cria variaveis globais
    if ArrayPontosSeg4:
        for x in range (0, len(ArrayPontosSeg4[0][0])):
                globals()['arraySomaSeg4%s' % x] = []
        #atribui a
        for i in range(0,len(ArrayPontosSeg4)): 
            for x in range (0, len(ArrayPontosSeg4[0][0])):
                globals()['arraySomaSeg4%s' % x].append(ArrayPontosSeg4[i][0][x])    
        #printa as variaveis:
        for x in range (0, len(ArrayPontosSeg4[0][0])):
            SomaDPSeg4 = SomaDPSeg4 + np.std(globals()['arraySomaSeg4%s' % x], ddof = 1)
            print(np.std(globals()['arraySomaSeg4%s' % x], ddof = 1))
        SomaDPSeg4  = SomaDPSeg4/(len(ArrayPontosSeg4[0][0]))
    else:
        SomaDPSeg4 = 0


    somaDPSegs = []
    somaDPSegs.append(SomaDPSeg1)
    somaDPSegs.append(SomaDPSeg2)
    somaDPSegs.append(SomaDPSeg3)
    somaDPSegs.append(SomaDPSeg4 -  0.02)


    somarAllSeg = SomaDPSeg1 + SomaDPSeg2 + SomaDPSeg3+SomaDPSeg4

    print ("_________Soma dos Seguimentos_____________")
    print('Seguimento 1 -- > %s' % SomaDPSeg1)
    print('Seguimento 2 -- > %s' % SomaDPSeg2)
    print('Seguimento 3 -- > %s' % SomaDPSeg3)
    print('Seguimento 4 -- > %s' % (SomaDPSeg4-  0.02))

    print ("______________________")
    maior = 0


    for i in range(0,len(somaDPSegs)):        
        if somaDPSegs[i] > maior:
            maior = somaDPSegs[i]

    teste = (somarAllSeg/6)+ ((somarAllSeg/6)* 0.2)
    print(somarAllSeg)
 
    for i in range(0,len(somaDPSegs)):
        
        if somaDPSegs[i] == maior:  
            
            if i == 0:
               print('Mao Esquerda moveu -- > %s' % somaDPSegs[i])
               rows = rowslefthand
               ehLeftHand = True
            if i == 1:
                print('Mao Direita se moveu -- > %s' % somaDPSegs[i])
                rows = rowsrighthand
                ehRightHand = True
            if i == 2:
                rows = rowsbocalist
                ehBoca = True
                print('Boca se moveu -- > %s' % somaDPSegs[i])
            if i == 3:
                rows = rowsrostolist
                ehRosto = True
                print('Cabeca se moveu -- > %s' % somaDPSegs[i])
            # if i == 4:
            #     rows = rowsbracodireitolist
            #     EhBracoDireito = True
            #     print('Baraco Direito se moveu -- > %s' % somaDPSegs[i])
            # if i == 5:
            #     rows = rowsbracoesquerdolist
            #     EhBracoEsquerdo
            #     print('Baraco Esquerdo se moveu -- > %s' % somaDPSegs[i])
 


def start():
    global  pararGravar
    global start_time
    global leftHandHuman,rightHandHuman,lefthandhumanarray,righthandhumanarray
    global boca_listarray,rosto_listarray,bracodireito_listarray,bracoesquerdo_listarray
    global classe
    
    leftHandHuman = []
    rightHandHuman = []
    #lefthandhumanarray = []
    
    print("O Teste comeca em:")
    for i in range(1,6):
        time.sleep(1)
        print('%s segundo' % (6-i))    
        if i == 5:
            start_time = time.time()
            print("Gravando...")
            pararGravar = False

def resetVars():
    global lefthandhumanarray,righthandhumanarray
    global boca_listarray,rosto_listarray,bracodireito_listarray,bracoesquerdo_listarray
    global ehLeftHand,ehRightHand,ehBoca,ehRosto,EhBracoDireito,EhBracoEsquerdo
    global rows, rowslefthand,rowsrighthand,rowsbocalist,rowsrostolist    
    global leftHandHumanInit,leftHandHumanFim,rightHandHumanInit,rightHandHumanFim,leftHandHumanOld,rightHandHumanOld,face_mesh_list_old,face_mesh_list_Fim,face_mesh_list_Init,pose_list_Init,pose_list_Fim
    global face_mesh_list,pose_list_old,pose_list,ArrayAllPontos,ArrayPontosSeg1,ArrayPontosSeg2,ArrayPontosSeg3,ArrayPontosSeg4,ArrayPontosSeg5,ArrayPontosSeg6,ArrayPontosSeg7

    rows = []
    rowslefthand = []
    rowsrighthand = []
    rowsbocalist = []
    rowsrostolist = []
    leftHandHumanInit = []
    leftHandHumanFim = []
    rightHandHumanInit= []
    rightHandHumanFim= []
    face_mesh_list_Fim = []
    face_mesh_list_Init = []
    leftHandHumanOld = []
    rightHandHumanOld = []
    leftHandHuman = []
    rightHandHuman = []
    face_mesh_list_old = []
    face_mesh_list = []
    pose_list_old = []
    pose_list = []
    ArrayAllPontos = []
    ArrayPontosSeg1 = []
    ArrayPontosSeg2 = []
    ArrayPontosSeg3 = []
    ArrayPontosSeg4 = []
    ArrayPontosSeg5 = []
    ArrayPontosSeg6 = []
    ArrayPontosSeg7 = []
    lefthandhumanarray = []
    righthandhumanarray = []
    boca_listarray = []
    rosto_listarray = []
    bracodireito_listarray = []
    bracoesquerdo_listarray = []

    ehLeftHand = False
    ehRightHand = False
    ehBoca = False
    ehRosto = False
    EhBracoDireito = False
    EhBracoEsquerdo = False

def frente():
    global classe
    classe = "frente"
    resetVars()
    start()
    
def tras():
    global classe
    classe = "tras"
    resetVars()
    start()

def direita():
    global classe
    classe = "direita"
    resetVars()
    start()

def esquerda():
    global classe
    classe = "esquerda"
    resetVars()
    start()

def updown():
    global classe
    classe = "updown"
    resetVars()
    start()

def efetuador():
    global classe
    classe = "efetuador"
    resetVars()
    start()

def calculaDifAngularDaReferencia2(p1,p2):
	#Calcula a diferenca angular
    #dist = np.linalg.norm(dados[0]-dados[1])
    #print(dist)
    pa = np.array(p1)
    pb = np.array(p2)  
    dist = np.linalg.norm(pa-pb)
   
    # distanciaEuclidiana = math.sqrt(((dados[0][0]-dados[0][1])**2) + ((dados[1][0] - dados[1][1])**2)  )
    # difAngular = math.atan2(dados[1][0]-dados[0][0], dados[1][1]-dados[0][1])
    # difAngular = (math.degrees(difAngular))
    # difAngular = (difAngular +360)%360 # para ele ir de 0 a 360
    return (dist)

def calculaDifAngularDaReferencia(dados):
	#Calcula a diferenca angular
    #dist = np.linalg.norm(dados[0]-dados[1])
    #print(dist)
    a = np.array((dados[0][0] ,dados[0][1]))
    b = np.array((dados[1][0], dados[1][1]))
    dist = np.linalg.norm(a-b)
    distanciaEuclidiana = math.sqrt(((dados[0][0]-dados[0][1])**2) + ((dados[1][0] - dados[1][1])**2)  )
    difAngular = math.atan2(dados[1][0]-dados[0][0], dados[1][1]-dados[0][1])
    difAngular = (math.degrees(difAngular))
    difAngular = (difAngular +360)%360 # para ele ir de 0 a 360
    return (dist)


def stop():
    global ehLeftHand,ehRightHand,ehBoca,ehRosto,EhBracoDireito,EhBracoEsquerdo
    global  pararGravar
    global lefthandhumanarray,righthandhumanarray,boca_listarray,rosto_listarray,bracodireito_listarray,bracoesquerdo_listarray
    global rows
    geral = 0
#LeftHand
    if ehLeftHand:
        lefthandarray = []
        #calcula ponto de maior ativacao levando em consideracao os pontos no tempo
        print(len(lefthandhumanarray))
        for i in range(1,len(lefthandhumanarray[0])):
            lefthand = [p[i-1:i] for p in lefthandhumanarray]
            lefthandarray. append(lefthand)

        distmaior = 0
        ponto = 0
        for p in range(0,len(lefthandarray)):
            dist = 0
            for i in range(1,len(lefthandarray[p])):
                dist = dist+ calculaDifAngularDaReferencia2(lefthandarray[p][i-1],lefthandarray[p][i]) 

            dist = dist/len(lefthandarray[p])
            if dist > distmaior:
                distmaior = dist
                ponto = p
        print(ponto)
        #_______________________________________________
        
        #Encontra maior distancia em x ou y do ponto de maior ativacao
        x1maior = 0
        x2maior = 0
        y1maior = 0
        y2maior = 0
        vx1 = lefthandhumanarray[0][ponto][0]
        vx2 = lefthandhumanarray[len(lefthandhumanarray)-1][ponto][0]
        vy1 = lefthandhumanarray[0][ponto][1]
        vy2 = lefthandhumanarray[len(lefthandhumanarray)-1][ponto][1]

        if vx1 > vx2:
        
            x2maior = 1000
            x1maior = -1000 
            for i in range(0,len(lefthandhumanarray)):
                x1 = lefthandhumanarray[i][ponto][0]
                x2 = lefthandhumanarray[i][ponto][0]
                if x1 > x1maior:
                    x1maior = x1
                if x2 < x2maior:
                    x2maior  = x2
        else:
            x1maior = 1000
            x2maior = -1000
            for i in range(0,len(lefthandhumanarray)):
                x1 = lefthandhumanarray[i][ponto][0]
                x2 = lefthandhumanarray[i][ponto][0]
                if x1 < x1maior:
                    x1maior = x1
                if x2 > x2maior:
                    x2maior  = x2

        if vy1 > vy2:

            y2maior = 1000
            y1maior = -1000
            for i in range(0,len(lefthandhumanarray)):
                y1 = lefthandhumanarray[i][ponto][1]
                y2 = lefthandhumanarray[i][ponto][1]
                if y1 > y1maior:
                    y1maior = y1
                if y2 < y2maior:
                    y2maior  = y2
        else:
            y1maior = 1000
            y2maior = -1000
            for i in range(0,len(lefthandhumanarray)):
                y1 = lefthandhumanarray[i][ponto][1]
                y2 = lefthandhumanarray[i][ponto][1]
                if y1 < y1maior:
                    y1maior = y1
                if y2 > y2maior:
                    y2maior  = y2         

        distx = np.linalg.norm(x1maior-x2maior)

        Xinicio = x1maior
        Xfim =  x2maior

        disty = np.linalg.norm(y1maior-y2maior)

        Yinicio =  y1maior
        Yfim =  y2maior


        pararGravar = True
    
        for row in rows:
            row.insert(0, ponto)
            if distx > disty:
            # print("Entrou em X")
                row.insert(1, 0)
                row.insert(2, Xinicio)
                row.insert(3, Xfim)
            else:
            # print("Entrou em Y")
                row.insert(1, 1)
                row.insert(2, y1maior)
                row.insert(3, y2maior)

            row.insert(4, classe)
            row.insert(5, 0)
            i = 42+8+10+6+6
            x = 49 
            while i > 0:
                row.insert(x , 0.0)
                i= i -1
                x = x +1
           
            with open('hand_dataset.csv', mode='a') as f:
                csv_writer = csv.writer(f, delimiter=',', quotechar='"', quoting=csv.QUOTE_MINIMAL)
                csv_writer.writerow(row)
        print("aqui salvaria")
        repeticao = 0
        rows = []

#RIGHT HAND++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
    if ehRightHand:
        righthandarray = []
        #calcula ponto de maior ativacao levando em consideracao os pontos no tempo
        print(len(righthandhumanarray))
        for i in range(1,len(righthandhumanarray[0])):
            righthand = [p[i-1:i] for p in righthandhumanarray]
            righthandarray. append(righthand)

        distmaior = 0
        ponto = 0
        for p in range(0,len(righthandarray)):
            dist = 0
            for i in range(1,len(righthandarray[p])):
                dist = dist+ calculaDifAngularDaReferencia2(righthandarray[p][i-1],righthandarray[p][i]) 

            dist = dist/len(righthandarray[p])
            if dist > distmaior:
                distmaior = dist
                ponto = p
        print(ponto)

        #_______________________________________________
        
        #Encontra maior distancia em x ou y do ponto de maior ativacao
        x1maior = 0
        x2maior = 0
        y1maior = 0
        y2maior = 0
        vx1 = righthandhumanarray[0][ponto][0]
        vx2 = righthandhumanarray[len(righthandhumanarray)-1][ponto][0]
        vy1 = righthandhumanarray[0][ponto][1]
        vy2 = righthandhumanarray[len(righthandhumanarray)-1][ponto][1]

        if vx1 > vx2:
        
            x2maior = 1000
            x1maior = -1000 
            for i in range(0,len(righthandhumanarray)):
                x1 = righthandhumanarray[i][ponto][0]
                x2 = righthandhumanarray[i][ponto][0]
                if x1 > x1maior:
                    x1maior = x1
                if x2 < x2maior:
                    x2maior  = x2
        else:
            x1maior = 1000
            x2maior = -1000
            for i in range(0,len(righthandhumanarray)):
                x1 = righthandhumanarray[i][ponto][0]
                x2 = righthandhumanarray[i][ponto][0]
                if x1 < x1maior:
                    x1maior = x1
                if x2 > x2maior:
                    x2maior  = x2

        if vy1 > vy2:

            y2maior = 1000
            y1maior = -1000
            for i in range(0,len(righthandhumanarray)):
                y1 = righthandhumanarray[i][ponto][1]
                y2 = righthandhumanarray[i][ponto][1]
                if y1 > y1maior:
                    y1maior = y1
                if y2 < y2maior:
                    y2maior  = y2
        else:
            y1maior = 1000
            y2maior = -1000
            for i in range(0,len(righthandhumanarray)):
                y1 = righthandhumanarray[i][ponto][1]
                y2 = righthandhumanarray[i][ponto][1]
                if y1 < y1maior:
                    y1maior = y1
                if y2 > y2maior:
                    y2maior  = y2

            

    


        distx = np.linalg.norm(x1maior-x2maior)

        Xinicio = x1maior
        Xfim =  x2maior

        disty = np.linalg.norm(y1maior-y2maior)

        Yinicio =  y1maior
        Yfim =  y2maior


        pararGravar = True
    
        for row in rows:
            row.insert(0, ponto)
            if distx > disty:
            # print("Entrou em X")
                row.insert(1, 0)
                row.insert(2, Xinicio)
                row.insert(3, Xfim)
            else:
            # print("Entrou em Y")
                row.insert(1, 1)
                row.insert(2, y1maior)
                row.insert(3, y2maior)
            row.insert(4, classe)
            row.insert(5, 1)
            i = 42
            x = 6
            while i > 0:
                row.insert(x , 0.0)
                i= i -1
                x = x +1

            q = 8+10+6+6 #numero de zero depois 
            y = x + 42
            while q > 0:
                row.insert(y, 0.0)
                q = q -1
                y = y +1

            with open('hand_dataset.csv', mode='a') as f:
                csv_writer = csv.writer(f, delimiter=',', quotechar='"', quoting=csv.QUOTE_MINIMAL)
                csv_writer.writerow(row)
        print("aqui salvaria")
        repeticao = 0
        rows = []

#BOCA+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
    if ehBoca:
        bocaarray = []
        
            #calcula ponto de maior ativacao levando em consideracao os pontos no tempo
        print(len(boca_listarray))
        for i in range(1,len(boca_listarray[0])):
            bocalist = [p[i-1:i] for p in boca_listarray]
            bocaarray. append(bocalist)

        distmaior = 0
        ponto = 0
        for p in range(0,len(bocaarray)):
            dist = 0
            for i in range(1,len(bocaarray[p])):
                dist = dist+ calculaDifAngularDaReferencia2(bocaarray[p][i-1],bocaarray[p][i]) 

            dist = dist/len(bocaarray[p])
            if dist > distmaior:
                distmaior = dist
                ponto = p
        print(ponto)
            #_______________________________________________
            
            #Encontra maior distancia em x ou y do ponto de maior ativacao
        x1maior = 0
        x2maior = 0
        y1maior = 0
        y2maior = 0
        vx1 = boca_listarray[0][ponto][0]
        vx2 = boca_listarray[len(boca_listarray)-1][ponto][0]
        vy1 = boca_listarray[0][ponto][1]
        vy2 = boca_listarray[len(boca_listarray)-1][ponto][1]

        if vx1 > vx2:
            
            x2maior = 1000
            x1maior = -1000 
            for i in range(0,len(boca_listarray)):
                x1 = boca_listarray[i][ponto][0]
                x2 = boca_listarray[i][ponto][0]
                if x1 > x1maior:
                    x1maior = x1
                if x2 < x2maior:
                    x2maior  = x2
        else:
            x1maior = 1000
            x2maior = -1000
            for i in range(0,len(boca_listarray)):
                x1 = boca_listarray[i][ponto][0]
                x2 = boca_listarray[i][ponto][0]
                if x1 < x1maior:
                    x1maior = x1
                if x2 > x2maior:
                    x2maior  = x2

        if vy1 > vy2:

            y2maior = 1000
            y1maior = -1000
            for i in range(0,len(boca_listarray)):
                y1 = boca_listarray[i][ponto][1]
                y2 = boca_listarray[i][ponto][1]
                if y1 > y1maior:
                    y1maior = y1
                if y2 < y2maior:
                    y2maior  = y2
        else:
            y1maior = 1000
            y2maior = -1000
            for i in range(0,len(boca_listarray)):
                y1 = boca_listarray[i][ponto][1]
                y2 = boca_listarray[i][ponto][1]
                if y1 < y1maior:
                    y1maior = y1
                if y2 > y2maior:
                    y2maior  = y2

        distx = np.linalg.norm(x1maior-x2maior)

        Xinicio = x1maior
        Xfim =  x2maior

        disty = np.linalg.norm(y1maior-y2maior)

        Yinicio =  y1maior
        Yfim =  y2maior


        pararGravar = True
        
        for row in rows:
            if ponto == 1:
                row.insert(0, 17)
            if ponto == 2:
                row.insert(0, 61)
            if ponto == 3:
                row.insert(0, 291)
            if distx > disty:
                # print("Entrou em X")
                row.insert(1, 0)
                row.insert(2, Xinicio)
                row.insert(3, Xfim)
            else:
                # print("Entrou em Y")
                row.insert(1, 1)
                row.insert(2, y1maior)
                row.insert(3, y2maior)
            row.insert(4, classe)
            row.insert(5, 2)
            i = 42+42
            x = 6
            while i > 0:
                row.insert(x , 0.0)
                i= i -1
                x = x +1
            q = 10+6+6 #numero de zero depois 
            y = x + 8
            while q > 0:
                row.insert(y, 0.0)
                q = q -1
                y = y +1
            with open('hand_dataset.csv', mode='a') as f:
                csv_writer = csv.writer(f, delimiter=',', quotechar='"', quoting=csv.QUOTE_MINIMAL)
                csv_writer.writerow(row)
        print("aqui salvaria")
        repeticao = 0
        rows = []
#ROSTO+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
    if ehRosto:
        rostoarray = []
        
            #calcula ponto de maior ativacao levando em consideracao os pontos no tempo
        print(len(rosto_listarray))
        for i in range(1,len(rosto_listarray[0])):
            rostolist = [p[i-1:i] for p in rosto_listarray]
            rostoarray. append(rostolist)

        distmaior = 0
        ponto = 0
        for p in range(0,len(rostoarray)):
            dist = 0
            for i in range(1,len(rostoarray[p])):
                dist = dist+ calculaDifAngularDaReferencia2(rostoarray[p][i-1],rostoarray[p][i]) 

            dist = dist/len(rostoarray[p])
            if dist > distmaior:
                distmaior = dist
                ponto = p
        print(ponto)
            #_______________________________________________
            
            #Encontra maior distancia em x ou y do ponto de maior ativacao
        x1maior = 0
        x2maior = 0
        y1maior = 0
        y2maior = 0
        vx1 = rosto_listarray[0][ponto][0]
        vx2 = rosto_listarray[len(rosto_listarray)-1][ponto][0]
        vy1 = rosto_listarray[0][ponto][1]
        vy2 = rosto_listarray[len(rosto_listarray)-1][ponto][1]

        if vx1 > vx2:
            
            x2maior = 1000
            x1maior = -1000 
            for i in range(0,len(rosto_listarray)):
                x1 = rosto_listarray[i][ponto][0]
                x2 = rosto_listarray[i][ponto][0]
                if x1 > x1maior:
                    x1maior = x1
                if x2 < x2maior:
                    x2maior  = x2
        else:
            x1maior = 1000
            x2maior = -1000
            for i in range(0,len(rosto_listarray)):
                x1 = rosto_listarray[i][ponto][0]
                x2 = rosto_listarray[i][ponto][0]
                if x1 < x1maior:
                    x1maior = x1
                if x2 > x2maior:
                    x2maior  = x2

        if vy1 > vy2:

            y2maior = 1000
            y1maior = -1000
            for i in range(0,len(rosto_listarray)):
                y1 = rosto_listarray[i][ponto][1]
                y2 = rosto_listarray[i][ponto][1]
                if y1 > y1maior:
                    y1maior = y1
                if y2 < y2maior:
                    y2maior  = y2
        else:
            y1maior = 1000
            y2maior = -1000
            for i in range(0,len(rosto_listarray)):
                y1 = rosto_listarray[i][ponto][1]
                y2 = rosto_listarray[i][ponto][1]
                if y1 < y1maior:
                    y1maior = y1
                if y2 > y2maior:
                    y2maior  = y2

        distx = np.linalg.norm(x1maior-x2maior)

        Xinicio = x1maior
        Xfim =  x2maior

        disty = np.linalg.norm(y1maior-y2maior)

        Yinicio =  y1maior
        Yfim =  y2maior


        pararGravar = True

        for row in rows:
            if ponto == 0:
                row.insert(0, 0)
            if ponto == 1:
                row.insert(0, 152)
            if ponto == 2:
                row.insert(0, 234)
            if ponto == 3:
                row.insert(0, 10)
            if ponto == 4:
                row.insert(0, 454)
            if distx > disty:
                # print("Entrou em X")
                row.insert(1, 0)
                row.insert(2, Xinicio)
                row.insert(3, Xfim)
            else:
                # print("Entrou em Y")
                row.insert(1, 1)
                row.insert(2, y1maior)
                row.insert(3, y2maior)
            row.insert(4, classe)
            row.insert(5, 4)
            i = 42+42+8
            x = 6
            while i > 0:
                row.insert(x , 0.0)
                i= i -1
                x = x +1
            q = 6 + 6 #numero de zero depois 
            y = x + 10
            while q > 0:
                row.insert(y, 0.0)
                q = q -1
                y = y +1
            with open('hand_dataset.csv', mode='a') as f:
                csv_writer = csv.writer(f, delimiter=',', quotechar='"', quoting=csv.QUOTE_MINIMAL)
                csv_writer.writerow(row)
        print("aqui salvaria")
        repeticao = 0
        rows = []




def getData():
    global leftHandHuman, rightHandHuman, face_mesh_list,pose_list
    leftHandHuman = []
    rightHandHuman = []
    face_mesh_list = []
    pose_list = []
    msg = rospy.wait_for_message("/MediaPipePose/holistic/landmarks", MediaPipeHolistic)
   
    if (msg.left_hand_landmarks):
        for i in range(0,len(msg.left_hand_landmarks)):
            leftHandHuman.append(msg.left_hand_landmarks[i])
                
    if (msg.right_hand_landmarks):
        for i in range(0,len(msg.right_hand_landmarks)):
            rightHandHuman.append(msg.right_hand_landmarks[i])
    
    if (msg.face_landmarks):
        for i in range(0,len(msg.face_landmarks)):
            face_mesh_list.append(msg.face_landmarks[i])
         
    if (msg.pose_landmarks):
        for i in range(0,len(msg.pose_landmarks)):
            pose_list.append(msg.pose_landmarks[i])

def DiffAngLeftHand():
    global leftHandHuman
    global leftHandHumanAng

    leftHandHumanAng = []
    v = 0
    count = 1
    for i in range(0, len(leftHandHuman)-1):
        
        difAngular = math.atan2(leftHandHuman[i+1].x-leftHandHuman[v].x, leftHandHuman[i+1].y-leftHandHuman[v].y)
        difAngular = (math.degrees(difAngular))
        leftHandHumanAng.append(difAngular)
        if count < 3:
            v = i
            count = count + 1
        else:
            v=0
            count = 1
    return leftHandHumanAng
    

def GravarMovimento():
    global leftHandHuman,rightHandHuman,lefthandhumanarray,righthandhumanarray
    global classe, rows, rowslefthand,rowsrighthand,rowsbocalist, rowsright
    global leftHandHuman,face_mesh_list,rightHandHuman,pose_list
    global pontoReferenciaSeg1,pontoReferenciaSeg2,pontoReferenciaSeg3,pontoReferenciaSeg4,pontoReferenciaSeg5
    global ArrayPontos, pararGravar
    global leftHandHumanOld,rightHandHumanOld,face_mesh_list_old,pose_list_old
    global leftHandHumanFim, leftHandHumanInit,rightHandHumanFim, rightHandHumanInit,face_mesh_list_Fim,face_mesh_list_Init
    
    distanciasSegmento1 = []
    distanciasSegmento2 = []
    distanciasSegmento3 = []
    distanciasSegmento4 = []
    distanciasSegmento5 = []
    distanciasSegmento6 = []
    distanciasSegmento7 = []

    #segmento 1 mao esquerda
    
    if leftHandHuman:

        lefthandx = [p.x for p in leftHandHuman]
        lefthandy = [p.y for p in leftHandHuman]
        centroidlefthand = (lefthandx[0],lefthandy[0])
        lefthandxy = [(p.x - centroidlefthand[0],p.y - centroidlefthand[1]) for p in leftHandHuman]

        lefthandhumanarray.append(lefthandxy)
       
        lefthandrow = list(np.array([[lhh.x - centroidlefthand[0], lhh.y - centroidlefthand[1]] for lhh in leftHandHuman]).flatten())
        
        # Concate rows
        rowslefthand.append(lefthandrow) 
 
        if not leftHandHumanOld:
            leftHandHumanInit = leftHandHuman
            leftHandHumanOld = leftHandHuman

        referenciaSeg1 = [leftHandHumanOld[0].x, leftHandHumanOld[0].y]
        distSeg1Pt0to1 = calculaDifAngularDaReferencia([referenciaSeg1,
        [leftHandHuman[1].x, leftHandHuman[1].y]])

       # referenciaSeg1 = [leftHandHumanOld[2].x, leftHandHumanOld[2].y]
        distSeg1Pt0to2 = calculaDifAngularDaReferencia([referenciaSeg1,
        [leftHandHuman[2].x, leftHandHuman[2].y]])

        #referenciaSeg1 = [leftHandHumanOld[3].x, leftHandHumanOld[3].y]
        distSeg1Pt0to3 = calculaDifAngularDaReferencia([referenciaSeg1,
        [leftHandHuman[3].x, leftHandHuman[3].y]])

        #referenciaSeg1 = [leftHandHumanOld[4].x, leftHandHumanOld[4].y]
        distSeg1Pt0to4 = calculaDifAngularDaReferencia([referenciaSeg1,
        [leftHandHuman[4].x, leftHandHuman[4].y]])
        

       # referenciaSeg1 = [leftHandHumanOld[5].x, leftHandHumanOld[5].y]
        distSeg1Pt0to5 = calculaDifAngularDaReferencia([referenciaSeg1,
        [leftHandHuman[5].x, leftHandHuman[5].y]])

        #referenciaSeg1 = [leftHandHumanOld[6].x, leftHandHumanOld[6].y]
        distSeg1Pt0to6 = calculaDifAngularDaReferencia([referenciaSeg1,
        [leftHandHuman[6].x, leftHandHuman[6].y]])

        #referenciaSeg1 = [leftHandHumanOld[7].x, leftHandHumanOld[7].y]
        distSeg1Pt0to7 = calculaDifAngularDaReferencia([referenciaSeg1,
        [leftHandHuman[7].x, leftHandHuman[7].y]])

       # referenciaSeg1 = [leftHandHumanOld[8].x, leftHandHumanOld[8].y]
        distSeg1Pt0to8 = calculaDifAngularDaReferencia([referenciaSeg1,
        [leftHandHuman[8].x, leftHandHuman[8].y]])

    

       # referenciaSeg1 = [leftHandHumanOld[9].x, leftHandHumanOld[9].y]
        distSeg1Pt0to9 = calculaDifAngularDaReferencia([referenciaSeg1,
        [leftHandHuman[9].x, leftHandHuman[9].y]])

       # referenciaSeg1 = [leftHandHumanOld[10].x, leftHandHumanOld[10].y]
        distSeg1Pt0to10 = calculaDifAngularDaReferencia([referenciaSeg1,
        [leftHandHuman[10].x, leftHandHuman[10].y]])

        #referenciaSeg1 = [leftHandHumanOld[11].x, leftHandHumanOld[11].y]
        distSeg1Pt0to11 = calculaDifAngularDaReferencia([referenciaSeg1,
        [leftHandHuman[11].x, leftHandHuman[11].y]])

       # referenciaSeg1 = [leftHandHumanOld[12].x, leftHandHumanOld[12].y]
        distSeg1Pt0to12 = calculaDifAngularDaReferencia([referenciaSeg1,
        [leftHandHuman[12].x, leftHandHuman[12].y]])


       # referenciaSeg1 = [leftHandHumanOld[13].x, leftHandHumanOld[13].y]
        distSeg1Pt0to13 = calculaDifAngularDaReferencia([referenciaSeg1,
        [leftHandHuman[13].x, leftHandHuman[13].y]])

       # referenciaSeg1 = [leftHandHumanOld[14].x, leftHandHumanOld[14].y]
        distSeg1Pt0to14 = calculaDifAngularDaReferencia([referenciaSeg1,
        [leftHandHuman[14].x, leftHandHuman[14].y]])

       # referenciaSeg1 = [leftHandHumanOld[15].x, leftHandHumanOld[15].y]
        distSeg1Pt0to15 = calculaDifAngularDaReferencia([referenciaSeg1,
        [leftHandHuman[15].x, leftHandHuman[15].y]])

       # referenciaSeg1 = [leftHandHumanOld[16].x, leftHandHumanOld[16].y]
        distSeg1Pt0to16 = calculaDifAngularDaReferencia([referenciaSeg1,
        [leftHandHuman[16].x, leftHandHuman[16].y]])


       # referenciaSeg1 = [leftHandHumanOld[17].x, leftHandHumanOld[17].y]
        distSeg1Pt0to17 = calculaDifAngularDaReferencia([referenciaSeg1,
        [leftHandHuman[17].x, leftHandHuman[17].y]])

       # referenciaSeg1 = [leftHandHumanOld[18].x, leftHandHumanOld[18].y]
        distSeg1Pt0to18 = calculaDifAngularDaReferencia([referenciaSeg1,
        [leftHandHuman[18].x, leftHandHuman[18].y]])

       # referenciaSeg1 = [leftHandHumanOld[19].x, leftHandHumanOld[19].y]
        distSeg1Pt0to19 = calculaDifAngularDaReferencia([referenciaSeg1,
        [leftHandHuman[19].x, leftHandHuman[19].y]])

       # referenciaSeg1 = [leftHandHumanOld[20].x, leftHandHumanOld[20].y]
        distSeg1Pt0to20 = calculaDifAngularDaReferencia([referenciaSeg1,
        [leftHandHuman[20].x, leftHandHuman[20].y]])

        leftHandHumanOld = leftHandHuman
        leftHandHumanFim = leftHandHuman

        distanciasSegmento1.append([
        distSeg1Pt0to1,
        distSeg1Pt0to2,
        distSeg1Pt0to3,
        distSeg1Pt0to4,
        distSeg1Pt0to5,
        distSeg1Pt0to6,
        distSeg1Pt0to7,
        distSeg1Pt0to8,
        distSeg1Pt0to9,
        distSeg1Pt0to10,
        distSeg1Pt0to11,
        distSeg1Pt0to12,
        distSeg1Pt0to13,
        distSeg1Pt0to14,
        distSeg1Pt0to15,
        distSeg1Pt0to16,
        distSeg1Pt0to17,
        distSeg1Pt0to18,
        distSeg1Pt0to19,
        distSeg1Pt0to20
        ])
        ArrayPontosSeg1.append(distanciasSegmento1)

    #segmento 2 mao direita
    if rightHandHuman:
        righthandx = [p.x for p in rightHandHuman]
        righthandy = [p.y for p in rightHandHuman]
        centroidrighthand = (righthandx[0],righthandy[0])
        righthandxy = [(p.x - centroidrighthand[0],p.y - centroidrighthand[1]) for p in rightHandHuman]
        righthandhumanarray.append(righthandxy)

        righthandrow = list(np.array([[lhh.x - centroidrighthand[0], lhh.y - centroidrighthand[1]] for lhh in rightHandHuman]).flatten())
        # Concate rows
        rowsrighthand.append(righthandrow)
        if not rightHandHumanOld:
            rightHandHumanInit = rightHandHuman
            rightHandHumanOld = rightHandHuman

        referenciaSeg2 = [rightHandHumanOld[1].x, rightHandHumanOld[1].y]
        distSeg2Pt0to1 = calculaDifAngularDaReferencia([referenciaSeg2,
        [rightHandHuman[1].x, rightHandHuman[1].y]])

        referenciaSeg2 = [rightHandHumanOld[2].x, rightHandHumanOld[2].y]
        distSeg2Pt0to2 = calculaDifAngularDaReferencia([referenciaSeg2,
        [rightHandHuman[2].x, rightHandHuman[2].y]])

        referenciaSeg2 = [rightHandHumanOld[3].x, rightHandHumanOld[3].y]
        distSeg2Pt0to3 = calculaDifAngularDaReferencia([referenciaSeg2,
        [rightHandHuman[3].x, rightHandHuman[3].y]])

        referenciaSeg2 = [rightHandHumanOld[4].x, rightHandHumanOld[4].y]
        distSeg2Pt0to4 = calculaDifAngularDaReferencia([referenciaSeg2,
        [rightHandHuman[4].x, rightHandHuman[4].y]])
    

        referenciaSeg2 = [rightHandHumanOld[5].x, rightHandHumanOld[5].y]
        distSeg2Pt0to5 = calculaDifAngularDaReferencia([referenciaSeg2,
        [rightHandHuman[5].x, rightHandHuman[5].y]])

        referenciaSeg2 = [rightHandHumanOld[6].x, rightHandHumanOld[6].y]
        distSeg2Pt0to6 = calculaDifAngularDaReferencia([referenciaSeg2,
        [rightHandHuman[6].x, rightHandHuman[6].y]])

        referenciaSeg2 = [rightHandHumanOld[7].x, rightHandHumanOld[7].y]
        distSeg2Pt0to7 = calculaDifAngularDaReferencia([referenciaSeg2,
        [rightHandHuman[7].x, rightHandHuman[7].y]])

        referenciaSeg2 = [rightHandHumanOld[8].x, rightHandHumanOld[8].y]
        distSeg2Pt0to8 = calculaDifAngularDaReferencia([referenciaSeg2,
        [rightHandHuman[8].x, rightHandHuman[8].y]])


        referenciaSeg2 = [rightHandHumanOld[9].x, rightHandHumanOld[9].y]
        distSeg2Pt0to9 = calculaDifAngularDaReferencia([referenciaSeg2,
        [rightHandHuman[9].x, rightHandHuman[9].y]])

        referenciaSeg2 = [rightHandHumanOld[10].x, rightHandHumanOld[10].y]
        distSeg2Pt0to10 = calculaDifAngularDaReferencia([referenciaSeg2,
        [rightHandHuman[10].x, rightHandHuman[10].y]])

        referenciaSeg2 = [rightHandHumanOld[11].x, rightHandHumanOld[11].y]
        distSeg2Pt0to11 = calculaDifAngularDaReferencia([referenciaSeg2,
        [rightHandHuman[11].x, rightHandHuman[11].y]])

        referenciaSeg2 = [rightHandHumanOld[12].x, rightHandHumanOld[12].y]
        distSeg2Pt0to12 = calculaDifAngularDaReferencia([referenciaSeg2,
        [rightHandHuman[12].x, rightHandHuman[12].y]])


        referenciaSeg2 = [rightHandHumanOld[13].x, rightHandHumanOld[13].y]
        distSeg2Pt0to13 = calculaDifAngularDaReferencia([referenciaSeg2,
        [rightHandHuman[13].x, rightHandHuman[13].y]])

        referenciaSeg2 = [rightHandHumanOld[14].x, rightHandHumanOld[14].y]
        distSeg2Pt0to14 = calculaDifAngularDaReferencia([referenciaSeg2,
        [rightHandHuman[14].x, rightHandHuman[14].y]])

        referenciaSeg2 = [rightHandHumanOld[15].x, rightHandHumanOld[15].y]
        distSeg2Pt0to15 = calculaDifAngularDaReferencia([referenciaSeg2,
        [rightHandHuman[15].x, rightHandHuman[15].y]])

        referenciaSeg2 = [rightHandHumanOld[16].x, rightHandHumanOld[16].y]
        distSeg2Pt0to16 = calculaDifAngularDaReferencia([referenciaSeg2,
        [rightHandHuman[16].x, rightHandHuman[16].y]])

        referenciaSeg2 = [rightHandHumanOld[17].x, rightHandHumanOld[17].y]
        distSeg2Pt0to17 = calculaDifAngularDaReferencia([referenciaSeg2,
        [rightHandHuman[17].x, rightHandHuman[17].y]])

        referenciaSeg2 = [rightHandHumanOld[18].x, rightHandHumanOld[18].y]
        distSeg2Pt0to18 = calculaDifAngularDaReferencia([referenciaSeg2,
        [rightHandHuman[18].x, rightHandHuman[18].y]])

        referenciaSeg2 = [rightHandHumanOld[19].x, rightHandHumanOld[19].y]
        distSeg2Pt0to19 = calculaDifAngularDaReferencia([referenciaSeg2,
        [rightHandHuman[19].x, rightHandHuman[19].y]])

        referenciaSeg2 = [rightHandHumanOld[20].x, rightHandHumanOld[20].y]
        distSeg2Pt0to20 = calculaDifAngularDaReferencia([referenciaSeg2,
        [rightHandHuman[20].x, rightHandHuman[20].y]])

        rightHandHumanOld = rightHandHuman
        rightHandHumanFim = rightHandHuman
        
        distanciasSegmento2.append([
            distSeg2Pt0to1,
            distSeg2Pt0to2,
            distSeg2Pt0to3,
            distSeg2Pt0to4,
            distSeg2Pt0to5,
            distSeg2Pt0to6,
            distSeg2Pt0to7,
            distSeg2Pt0to8,
            distSeg2Pt0to9,
            distSeg2Pt0to10,
            distSeg2Pt0to11,
            distSeg2Pt0to12,
            distSeg2Pt0to13,
            distSeg2Pt0to14,
            distSeg2Pt0to15,
            distSeg2Pt0to16,
            distSeg2Pt0to17,
            distSeg2Pt0to18,
            distSeg2Pt0to19,
            distSeg2Pt0to20
        ])
       
        ArrayPontosSeg2.append(distanciasSegmento2)
        print(ArrayPontosSeg2)

    #segmento 3 boca
    if face_mesh_list:
        boca_list = []
        boca_list.append(face_mesh_list[0])
        boca_list.append(face_mesh_list[17])
        boca_list.append(face_mesh_list[61])
        boca_list.append(face_mesh_list[2])
        

        bocalistx= [p.x for p in boca_list]
        bocalisty= [p.y for p in boca_list]
        centroidbocalist = (boca_list[0].x,boca_list[0].y)
        
        bocalistxy = [(p.x - centroidbocalist[0], p.y - centroidbocalist[1]) for p in boca_list]
        
        boca_listarray.append(bocalistxy)

        bocalistrow = list(np.array([[lhh.x - centroidbocalist[0], lhh.y - centroidbocalist[1]] for lhh in boca_list]).flatten())
        # Concate rows
        rowsbocalist.append(bocalistrow)

        if not face_mesh_list_old:
            face_mesh_list_Init = face_mesh_list
            face_mesh_list_old = face_mesh_list

        referenciaSeg3 = [face_mesh_list_old[0].x, face_mesh_list_old[0].y]
        distSeg3Pt0to314 = calculaDifAngularDaReferencia([referenciaSeg3,
        [face_mesh_list[17].x, face_mesh_list[17].y]])

        referenciaSeg3 = [face_mesh_list_old[0].x, face_mesh_list_old[0].y]
        distSeg3Pt0to61 = calculaDifAngularDaReferencia([referenciaSeg3,
        [face_mesh_list[61].x, face_mesh_list[61].y]])

        
        referenciaSeg3 = [face_mesh_list_old[0].x, face_mesh_list_old[0].y]
        distSeg3Pt0to291 = calculaDifAngularDaReferencia([referenciaSeg3,
        [face_mesh_list[291].x, face_mesh_list[291].y]])

        distanciasSegmento3.append([
            distSeg3Pt0to314,
            distSeg3Pt0to61,
            distSeg3Pt0to291

        ])
        ArrayPontosSeg3.append(distanciasSegmento3)       

        face_mesh_list_old = face_mesh_list
        face_mesh_list_Fim = face_mesh_list

        #segmento 4 rosto
        if pose_list:
            rosto_list = []
   
            rosto_list.append(face_mesh_list[0])
            rosto_list.append(face_mesh_list[152])
            rosto_list.append(face_mesh_list[234])
            rosto_list.append(face_mesh_list[10])
            rosto_list.append(face_mesh_list[454])          
          

            rosto_listx= [p.x for p in rosto_list]
            rosto_listy= [p.y for p in rosto_list]
            centx = (pose_list[12].x + pose_list[11].x) /2
            centy = (pose_list[12].y + pose_list[11].y) /2
            centroidrostolist = (centx,centy)
            
            rostolistxy = [(p.x - centroidrostolist[0], p.y - centroidrostolist[1]) for p in rosto_list]
            
            rosto_listarray.append(rostolistxy)

            rostolistrow = list(np.array([[lhh.x - centroidrostolist[0], lhh.y - centroidrostolist[1]] for lhh in rosto_list]).flatten())
            # Concate rows
            rowsrostolist.append(rostolistrow)

            referenciaSeg4 = [centx, centy]
            distSeg4Pt145to159 = calculaDifAngularDaReferencia([referenciaSeg4,
            [face_mesh_list[0].x, face_mesh_list[0].y]])

            distanciasSegmento4.append([
                distSeg4Pt145to159,
            ])
            ArrayPontosSeg4.append(distanciasSegmento4)
        
   
        ArrayPontosSeg5.append(distanciasSegmento5)
        ArrayPontosSeg6.append(distanciasSegmento6)






    
            
        


def init():
    global pararGravar
    global start_time
    global imagem_raw
    global leftHandHuman, rightHandHuman,PontosIniciaisME,PontosFinaisME
    global repeticao
    while not rospy.is_shutdown():
        ROOT.update()
        getData()
        if pararGravar == False:
            current_time = time.time()
            elapsed_time = current_time - start_time
            GravarMovimento()
            if elapsed_time > 3:
                calculaMaior()
                print("Gravacao Completa")

                repeticao = repeticao + 1
                if repeticao == 3:
                    stop()
                    repeticao = 0
                    rows = []
                else:
                    start()
            


ROOT = Tk()
LABEL = Label(ROOT, text="App Detector!")
frenteButton = Button(ROOT, height=2, width=20, text ="Forward", 
command = frente)
trasButton = Button(ROOT, height=2, width=20, text ="Backward", 
command = tras)
direitaButton = Button(ROOT, height=2, width=20, text ="Right", 
command = direita)
esquerdaButton = Button(ROOT, height=2, width=20, text ="Left", 
command = esquerda)
updownButton = Button(ROOT, height=2, width=20, text ="Up/Down", 
command = updown)
effectorButton = Button(ROOT, height=2, width=20, text ="Effector", 
command = efetuador)
stopButton = Button(ROOT, height=2, width=20, text ="stop", 
command = stop)
LABEL.pack()
frenteButton.pack()
trasButton.pack()
direitaButton.pack()
esquerdaButton.pack()
updownButton.pack()
effectorButton.pack()
stopButton.pack()

if __name__ == '__main__':
    rospy.init_node('mediapipe_geradb', anonymous=True)
    init()
    rospy.spin()