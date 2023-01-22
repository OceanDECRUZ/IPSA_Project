# -*- coding: utf-8 -*-
"""
Created on Tue Jan 10 15:20:22 2023

@author: matho
"""
from  matplotlib.animation import FuncAnimation
import numpy as np
import math as m
import matplotlib.pyplot as plt
import random as rd
N=20 #nombre de pixel en y
M=20 #nombre de prixel en x
n=3 #nombre d'information par cell


def InitGrid(N=N,M=M,n=n):
    G= np.zeros((N,M,n))
    return G

def InitEnv(G,human=[],obstacle=[],explored=[]):
    for i in human:
        G[i][0]=1    
    for i in obstacle:    
        G[i][1]=1
    for i in explored:
        G[i][2]=1
    return G

def WallToCell(x0,y0,x1,y1):
    C=[]
    for i in range(x0,x1+1):
        for j in range(y0,y1+1):
            C.append((i,j))
    return C

def contour(cell,n):
    #print(n)
    L=[]
    m=0
    for i in range(-n,n+1):
        for j in range(-n,n+1):
            m+=1
            good_contour=True
            if (i!=0 or j!=0) and 0<=i+cell[0]<N and 0<=j+cell[1]<M:
                if n==1:
                    L.append((cell[0]+i,cell[1]+j))
                else:
                                           
                    for k in range(1,n):                        
                        if ((i+cell[0],j+cell[1]) in contour(cell,k)):
                            good_contour=False
                            #print(i+cell[0],j+cell[1],contour(cell,k)) 
                        #print(k)
                    #print(L)
                    if good_contour:
                        L.append((cell[0]+i,cell[1]+j))
                        #print(L)
    return L

def first_contour(cell):
    L=[]
    for i in range(-1,2):
        for j in range(-1,2):
            if (i!=0 or j!=0) and 0<=i+cell[0]<N and 0<=j+cell[1]<M:
                L.append((cell[0]+i,cell[1]+j))
    return L

def second_contour(cell):
    L=[]
    fc=first_contour(cell)
    for i in range(-2,3):
        for j in range(-2,3):
            if (i!=0 or j!=0) and not((i+cell[0],j+cell[1]) in fc) and 0<=i+cell[0]<N and 0<=j+cell[1]<M:
                    L.append((cell[0]+i,cell[1]+j))
    return L

def third_contour(cell):
    L=[]
    fc=first_contour(cell)
    sc=second_contour(cell)
    for i in range(-3,4):
        for j in range(-3,4):
            if (i!=0 or j!=0) and not((i+cell[0],j+cell[1]) in fc) and not((i+cell[0],j+cell[1]) in sc) and 0<=i+cell[0]<N and 0<=j+cell[1]<M:
                    L.append((cell[0]+i,cell[1]+j))
    return L

def fourth_contour(cell):
    L=[]
    fc=first_contour(cell)
    sc=second_contour(cell)
    tc=third_contour(cell)
    for i in range(-4,5):
        for j in range(-4,5):
            if (i!=0 or j!=0) and not((i+cell[0],j+cell[1]) in tc) and not((i+cell[0],j+cell[1]) in fc) and not((i+cell[0],j+cell[1]) in sc) and 0<=i+cell[0]<N and 0<=j+cell[1]<M:
                    L.append((cell[0]+i,cell[1]+j))
    return L

def fifth_contour(cell):
    L=[]
    fc=first_contour(cell)
    sc=second_contour(cell)
    tc=third_contour(cell)
    foc=fourth_contour(cell)
    for i in range(-5,6):
        for j in range(-5,6):
            if (i!=0 or j!=0) and not((i+cell[0],j+cell[1]) in foc) and not((i+cell[0],j+cell[1]) in tc) and not((i+cell[0],j+cell[1]) in fc) and not((i+cell[0],j+cell[1]) in sc) and 0<=i+cell[0]<N and 0<=j+cell[1]<M:
                    L.append((cell[0]+i,cell[1]+j))
    return L

def ContourWall(Env,obs,N=N,M=M):
    G=InitGrid()
    for i in obs:
        for j in contour(i,1):
            if Env[j][1]!=1: 
                G[j][1]=0.5
    return G

def TempHum(Env,hum,N=N,M=M):
    G=InitGrid()
    for i in hum:
        for j in contour(i,1):
            if Env[j][1]!=1:
                if Env[j][0]!=1: 
                    G[j][0]=0.6
        for j in contour(i,2):
            if Env[j][1]!=1:
                k=m.floor(i[0]+((j[0]-i[0])/2))
                l=m.floor(i[1]+((j[1]-i[1])/2))
                if Env[j][0]!=1 and G[j][0]!=0.6 and Env[(k,l)][1]!=1:
                    G[j][0]=0.3
        for j in contour(i,3):
            if Env[j][1]!=1:
                k=i[0]+((j[0]-i[0])/2)
                l=i[1]+((j[1]-i[1])/2)
                if Env[j][0]!=1 and G[j][0]!=0.6 and G[j][0]!=0.3 and Env[(m.floor(k),m.floor(l))][1]!=1 and Env[(m.ceil(k),m.ceil(l))][1]!=1: 
                    G[j][0]=0.1
    return G

        
GridProba=InitGrid()

hum=[(0,8),(0,19),(2,4),(8,11),(8,17),(12,9),(19,8),(19,14),(19,19)]

obs=[WallToCell(2,3,17,3)+ WallToCell(1,15,5,15)+ WallToCell(2,19,6,19)+ WallToCell(15,7,19,7)+  WallToCell(14,12,18,12)+ WallToCell(15,16,19,16)+  WallToCell(11,18,15,18)+
             WallToCell(6,5,6,7)+  WallToCell(9,5,9,7)+  WallToCell(12,5,12,7)+  WallToCell(4,9,4,13)+  WallToCell(9,9,9,13)+  WallToCell(12,10,12,13)]
obs=obs[0]
exp=[]

GridProba=InitEnv(GridProba,hum,obs,exp)

plt.matshow(GridProba)

T=TempHum(GridProba,hum)

W=ContourWall(GridProba,obs)

GridProba+=T+W

plt.matshow(GridProba)
plt.matshow(W)
plt.matshow(T)

#%%

exp=[]

GridBelieves=InitGrid()+(1,1,0)

humain=[]

start=(18,1) #18,1 ou 0,17
wall_impact=True


def newpos(pos,GridBelieves,exp,GridProba=GridProba,start=start): #actualise la GridBelieves/la position
    exp.append(pos)
    humain_trouve=False    
    p=GridProba[exp[-1]]
    w_actu=[]
    h_actu=[]
    #Actualise la position estimée des murs

    if p[1]==0:
        w_actu=[0.5]
    elif p[1]==1:            
        print(pos,GridBelieves[exp[-1]])
        anim.pause()
    GridBelieves[exp[-1]]=p    
    GridBelieves[exp[-1]][2]=1
        
    for j in range(len(w_actu)):
        for i in contour(exp[-1],j):
            if GridBelieves[i][2]!=1:
                GridBelieves[i][1]=GridBelieves[i][1]*w_actu[j]
    
    #Actualise la position estimée des humains
    if p[0]==0.3:
        h_actu=[0.6]
    elif p[0]==0.1:
        h_actu=[0.3,0.6]
    elif p[0]==1:
        humain_trouve=True
    elif p[0]==0 and p[1]!=1:
        h_actu=[0.1,0.3,0.6]
    
    for j in range(len(h_actu)):
        for i in contour(exp[-1],j):
            if GridBelieves[i][2]!=1:
                GridBelieves[i][0]=GridBelieves[i][0]*h_actu[j]

    return exp,GridBelieves, humain_trouve


def best_dest(GridBelieves, exp): #décide la prochaine cellule
    L_dest=[]
    h_dest=[]
    w_dest=[]
    for i in contour(exp[-1],1):
        if GridBelieves[i][2]==0:
            if ((GridBelieves[i][1]==1 and GridBelieves[exp[-1]][1]==0) or GridBelieves[i][1]!=1):
                L_dest.append(i)
                h_dest.append(GridBelieves[i][0])
                w_dest.append(GridBelieves[i][1])
    if L_dest==[]:            
        for i in contour(exp[-1],1):          
            if GridBelieves[i][1]!=1:
                L_dest.append(i)           
        if L_dest==[]:
            print("Liste vide")
            anim.pause()
            return -1
        return rd.choice(L_dest)
    best_h=np.argmax(h_dest)
    return L_dest[best_h]

def newpos_wi(pos,GridBelieves,exp,start=start,GridProba=GridProba): #actualise la GridBelieves/la position

    exp.append(pos)
    p=GridProba[exp[-1]]
    GridBelieves[exp[-1]]=p    
    GridBelieves[exp[-1]][2]=1
    humain_trouve=False
    if p[0]==0.3:
        for i in contour(exp[-1],1): 
            if GridBelieves[i][2]!=1:
                GridBelieves[i][0]=GridBelieves[i][0]*0.6
        for i in contour(exp[-1],2): 
            if GridBelieves[i][2]!=1:
                GridBelieves[i][0]=GridBelieves[i][0]*1
        for i in contour(exp[-1],3):
            if GridBelieves[i][2]!=1:
                GridBelieves[i][0]=GridBelieves[i][0]*0.6
    elif p[0]==0.6:
        for i in contour(exp[-1],1): 
            if GridBelieves[i][2]!=1:
                GridBelieves[i][0]=GridBelieves[i][0]*1
        for i in contour(exp[-1],2): 
            if GridBelieves[i][2]!=1:
                GridBelieves[i][0]=GridBelieves[i][0]*0.6
        for i in contour(exp[-1],3):
            if GridBelieves[i][2]!=1:
                GridBelieves[i][0]=GridBelieves[i][0]*0.3
    elif p[0]==1:
        humain_trouve=True
        for i in contour(exp[-1],1): 
            if GridBelieves[i][2]!=1:
                GridBelieves[i][0]=GridBelieves[i][0]*0.6
        for i in contour(exp[-1],2): 
            if GridBelieves[i][2]!=1:
                GridBelieves[i][0]=GridBelieves[i][0]*0.3
    elif p[1]!=1:
        for i in contour(exp[-1],1): 
            if GridBelieves[i][2]!=1:
                GridBelieves[i][0]=GridBelieves[i][0]*0.3
        for i in contour(exp[-1],2): 
            if GridBelieves[i][2]!=1:
                GridBelieves[i][0]=GridBelieves[i][0]*0.6
        for i in contour(exp[-1],3):
            if GridBelieves[i][2]!=1:
                GridBelieves[i][0]=GridBelieves[i][0]*1
    if p[1]==0.5:
        for i in contour(exp[-1],1): 
            if GridBelieves[i][2]!=1:
                GridBelieves[i][1]=GridBelieves[i][1]*1
        for i in contour(exp[-1],2): 
            if GridBelieves[i][2]!=1:
                GridBelieves[i][1]=GridBelieves[i][1]*0.5
    elif p[1]==1:
        exp.append(exp[-2]) #recule
        #exp.append(start) #revient au départ
    else:
        for i in contour(exp[-1],1): 
            if GridBelieves[i][2]!=1:
                GridBelieves[i][1]=GridBelieves[i][1]*0.5
        for i in contour(exp[-1],2): 
            if GridBelieves[i][2]!=1:
                GridBelieves[i][1]=GridBelieves[i][1]*1
        for i in contour(exp[-1],3):
            if GridBelieves[i][2]!=1:
                GridBelieves[i][1]=GridBelieves[i][1]*0.5
    return exp,GridBelieves, humain_trouve

def best_dest_wi(GridBelieves, exp): #décide la prochaine cellule
    L_dest=[]
    h_dest=[]
    w_dest=[]
    for i in contour(exp[-1],1):
        if GridBelieves[i][2]==0:
            L_dest.append(i)
            h_dest.append(GridBelieves[i][0])
            w_dest.append(GridBelieves[i][1])
    if L_dest==[]:      
        for i in contour(exp[-1],1):
            if GridBelieves[i][1]!=1:
                L_dest.append(i)
        return rd.choice(L_dest)
    best_h=np.argmax(h_dest)
    while w_dest[best_h]==1:
        del h_dest[best_h]
        if h_dest==[]:
            return rd.choice(L_dest)
        best_h=np.argmax(h_dest)
    return L_dest[best_h]

def end_of_exploration(GridBelieves, N=N, M=M): #test si l'exploration est finie 
    Cell_non_expl=[]
    for i in range(N):
        for j in range(M):
            if GridBelieves[i,j,2]==0:
                Cell_non_expl.append((i,j))
    for i in range(len(Cell_non_expl)):
        for j in first_contour(Cell_non_expl[i]):
            if GridBelieves[j][2]==1 and GridBelieves[j][0]!=0.6 and GridBelieves[j][1]!=1 :
                Cell_non_expl[i]=False
                break
    if Cell_non_expl==[False for i in range(len(Cell_non_expl))] :
        return True
    return False
        

def update(j,GridBelieves,exp,humain,finish,wall_impact=wall_impact):

    if len(exp)>=210 and finish==[]:
        finish.append(0)
        plt.matshow(GridBelieves)
        print(len(humain),"humans have been found after 210 actions, there positions are: ", humain)
    if end_of_exploration(GridBelieves):
        print("\n*****************************************\nThe exploration end with ", len(exp), "actions.")
        print(len(humain),"humans have been found, there positions are: ", humain)
        anim.pause()
    else:
        if wall_impact:
            exp,GridBelieves, humain_trouve = newpos_wi(best_dest_wi(GridBelieves, exp),GridBelieves,exp)

        else:
            exp,GridBelieves, humain_trouve = newpos(best_dest(GridBelieves, exp),GridBelieves,exp)
        GridShown=GridBelieves.copy()
        GridShown[exp[-1]]=[1,0,0]
        plot.set_data(GridShown)
        if humain_trouve and exp[-1] not in humain:
            humain.append(exp[-1])
            if len(humain)==9:
                print("\nThe robot takes",len(exp),"actions to found all the humans.")
            else:
                print("The robot takes",len(exp),"actions to found",len(humain),"humans.")
    return [plot]

finish=[]
exp,GridBelieves, humain_trouve = newpos(start,GridBelieves,exp)
fig = plt.figure()
plot = plt.matshow(GridBelieves, fignum=0)

anim = FuncAnimation(fig, update, frames=5000, interval = 10, fargs=(GridBelieves,exp,humain,finish,), blit=True, repeat=False)

plt.show()
