
import numpy as np
from PIL import Image
from fonctions import create_video
import cv2
range_mur = [2,1.5,1,0.5]
range_human = [8,7,6]
valeur_mur = 4
valeur_human = 10
valeur_human_incertain = 9
mur_autour = 3
test = 50
dico_couleurs = {
        0:(255,255,255),
        test:(0,200,20),
        range_mur[0]:(60,0,60),
        range_mur[1]:(120,0,120),
        range_mur[2]:(180,0,180),
        range_mur[3]:(240,0,240),
        range_human[2]:(0,0,120),
        range_human[1]:(0,0,180),
        range_human[0]:(0,0,255),

        mur_autour:(180,100,100),
        valeur_mur: (160,0,0),
        valeur_human: (60,200,60),
        valeur_human_incertain:(120,40,40),
        
        -1:(80,80,80),
        -2:(160,160,160),
        -3:(200,240,200),
        5:(255,128,0),
        15:(128,128,20),
        }

dico_human = {
        0:(255,255,255), #blanc pour un truc vide
        test:(0,200,20),
        # range_human[2]:(0,0,120),
        # range_human[1]:(0,0,180),
        # range_human[0]:(0,0,255),
        valeur_human: (60,200,60),
        valeur_human_incertain:(120,40,40),
        5:(255,128,0),
        15:(128,128,20),
        -1:(80,80,80),
        valeur_mur: (160,0,0),
        }

dico_mur = {
        0:(255,255,255), #blanc pour un truc vide
        test:(0,200,20),
        range_mur[3]:(0,0,80),
        range_mur[2]:(0,0,120),
        range_mur[1]:(0,0,180),
        range_mur[0]:(0,0,255),
        valeur_mur: (160,0,0),
        5:(255,128,0),
        15:(128,128,20),
        -1:(80,80,80),
        valeur_human: (60,200,60),
        }

def display_single_map(map,couleurs,coef=40,taille_ligne=3):
    
    new_img = np.zeros([map.shape[0]*coef,coef*map.shape[1],3],dtype=np.uint8)
    new_certainty = np.zeros([map.shape[0]*coef,coef*map.shape[1],3],dtype=np.uint8)
    for i in range(map.shape[0]):
        for j in range(map.shape[1]):
            color=couleurs[map[i,j]]
            new_img[i*coef:(i+1)*coef-taille_ligne,j*coef:(j+1)*coef-1-taille_ligne]=color

    return Image.fromarray(new_img)

def display_nice_image(world,certainty,couleurs1,couleurs2,coef=40,taille_ligne=3):
    
    # Create a black image

    # Write some Text
    
    if world.shape[0]!=world.shape[1]:
        print("ne marche que pour des images carrées pour le moment")
        return world
    
    new_img = np.zeros([world.shape[0]*coef,coef*world.shape[1],3],dtype=np.uint8)
    new_certainty = np.zeros([world.shape[0]*coef,coef*world.shape[1],3],dtype=np.uint8)
    for i in range(world.shape[0]):
        
        for j in range(world.shape[1]):
            color=couleurs1[int(world[i,j])]
            new_img[i*coef:(i+1)*coef-taille_ligne,j*coef:(j+1)*coef-1-taille_ligne]=color
            color = couleurs2[certainty[i,j]]
            new_certainty[i*coef:(i+1)*coef-taille_ligne,j*coef:(j+1)*coef-1-taille_ligne]=color

            
            
    to_return = np.concatenate((new_img,new_certainty),axis=1)
    black_bottom= np.array([0]*1600*3*80).reshape((80,1600,3))
    to_return = np.concatenate((to_return,black_bottom),axis=0)
    # return Image.fromarray(np.concatenate((new_img,new_certainty),axis=1))
    return Image.fromarray(to_return.astype("uint8"))

def cherche_autour(position,referential_map,certainty_map_human,certainty_map_wall,value_to_find = -1,show_point_checked = False,depassement =[False,0]):
    ligne,colonne = position
    taille = 0
    # taille=int(taille)
    liste_point_autour = []
    list_point_checked = []
    while len(liste_point_autour)==0 and taille<max(referential_map.shape)+1:
        
        taille +=1
        for j in range(max(0,colonne-taille+1),min(referential_map.shape[1],colonne+taille),1):
            # print(f"{[ligne+taille,j]} and {[ligne-taille,j]} checked")
            if ligne-taille>=0:
                if referential_map[ligne-taille,j] == value_to_find and certainty_map_human[ligne-taille,j] not in [valeur_human,valeur_human_incertain] and certainty_map_wall[ligne-taille,j] not in [valeur_mur,mur_autour]:
                    liste_point_autour.append([ligne-taille,j])
                list_point_checked.append([ligne-taille,j])

            if ligne+taille<referential_map.shape[0]:
                if referential_map[ligne+taille,j] == value_to_find and certainty_map_human[ligne+taille,j] not in [valeur_human,valeur_human_incertain] and certainty_map_wall[ligne+taille,j] not in [valeur_mur,mur_autour]:
                    liste_point_autour.append([ligne+taille,j])
                list_point_checked.append([ligne+taille,j])
            
        for j in range(max(0,ligne-taille),min(referential_map.shape[0],ligne+taille+1),1):
            # print(f"{[j,colonne+taille]} and {[j,colonne-taille]} checked")
            if colonne+taille < referential_map.shape[1]:
                if referential_map[j,colonne+taille] == value_to_find and certainty_map_human[j,colonne+taille] not in [valeur_human,valeur_human_incertain] and certainty_map_wall[j,colonne+taille] not in [valeur_mur,mur_autour]:
                    liste_point_autour.append([j,colonne+taille])
                list_point_checked.append([j,colonne+taille])

            if colonne-taille >=0 :
                if referential_map[j,colonne-taille] == value_to_find and certainty_map_human[j,colonne-taille] not in [valeur_human,valeur_human_incertain] and certainty_map_wall[j,colonne-taille] not in [valeur_mur,mur_autour]:
                    liste_point_autour.append([j,colonne-taille])
                list_point_checked.append([j,colonne-taille])
    
    #depassement est inutile, mais je le garde au cas où...
    if depassement[0]:
        liste_point_autour = []
        list_point_checked = []

        taille +=depassement[1]

        if taille<max(referential_map.shape)+1:

            for j in range(max(0,colonne-taille+1),min(referential_map.shape[1],colonne+taille),1):
                if ligne-taille>=0:
                    if referential_map[ligne-taille,j] == value_to_find:
                        liste_point_autour.append([ligne-taille,j])
                    list_point_checked.append([ligne-taille,j])

                if ligne+taille<referential_map.shape[0]:
                    if referential_map[ligne+taille,j] == value_to_find:
                        liste_point_autour.append([ligne+taille,j])
                    list_point_checked.append([ligne+taille,j])
                
            for j in range(max(0,ligne-taille),min(referential_map.shape[0],ligne+taille+1),1):
                if colonne+taille < referential_map.shape[1]:
                    if referential_map[j,colonne+taille] == value_to_find:
                        liste_point_autour.append([j,colonne+taille])
                    list_point_checked.append([j,colonne+taille])

                if colonne-taille >=0 :
                    if referential_map[j,colonne-taille] == value_to_find:
                        liste_point_autour.append([j,colonne-taille])
                    list_point_checked.append([j,colonne-taille])


    if show_point_checked:
        return liste_point_autour,list_point_checked
    else :
        return liste_point_autour

def cherche_autourv2(position,referential_map,certainty_map_human,certainty_map_wall,value_to_find=-1,show_point_checked=False,taille=1,forastar = False):
    ligne,colonne = position
    # taille=int(taille)
    liste_point_autour = []
    list_point_checked = [] 
    if forastar:
        risk_hum = [valeur_human]
        risk_wal = [valeur_mur]
    else :
        risk_hum = [valeur_human,valeur_human_incertain]
        risk_wal = [valeur_mur,mur_autour]
    for j in range(max(0,colonne-taille+1),min(referential_map.shape[1],colonne+taille),1):
        # print(f"{[ligne+taille,j]} and {[ligne-taille,j]} checked")
        
        if ligne-taille>=0:
            
            if referential_map[ligne-taille,j] == value_to_find and certainty_map_human[ligne-taille,j] not in risk_hum and certainty_map_wall[ligne-taille,j] not in risk_wal:
                liste_point_autour.append([ligne-taille,j])
            list_point_checked.append([ligne-taille,j])

        if ligne+taille<referential_map.shape[0]:
            if referential_map[ligne+taille,j] == value_to_find and certainty_map_human[ligne+taille,j] not in risk_hum and certainty_map_wall[ligne+taille,j] not in risk_wal:
                liste_point_autour.append([ligne+taille,j])
            list_point_checked.append([ligne+taille,j])
        
    for j in range(max(0,ligne-taille),min(referential_map.shape[0],ligne+taille+1),1):
        # print(f"{[j,colonne+taille]} and {[j,colonne-taille]} checked")
        if colonne+taille < referential_map.shape[1]:
            if referential_map[j,colonne+taille] == value_to_find and certainty_map_human[j,colonne+taille] not in risk_hum and certainty_map_wall[j,colonne+taille] not in risk_wal:
                liste_point_autour.append([j,colonne+taille])
            list_point_checked.append([j,colonne+taille])

        if colonne-taille >=0 :
            if referential_map[j,colonne-taille] == value_to_find and certainty_map_human[j,colonne-taille] not in risk_hum and certainty_map_wall[j,colonne-taille] not in risk_wal:
                liste_point_autour.append([j,colonne-taille])
            list_point_checked.append([j,colonne-taille])
    if show_point_checked:
        return liste_point_autour,list_point_checked
    else :
        return liste_point_autour

def coef_priority(map,value,ligne,colonne,ordre_priorite):
    
    if (ligne<map.shape[0]) and (ligne>=0) and (colonne<map.shape[1]) and (colonne>=0):
        if ordre_priorite.index(map[ligne,colonne])>ordre_priorite.index(value):
            map[ligne,colonne] = value
    return map

def cercle_autourv2(map,position,taille,value,ordre_priorite):
    ligne,colonne = position
    taille = int(taille/2)
    # taille=int(taille)
    
    for j in range(colonne-taille+1,colonne+taille,1):
        map = coef_priority(map,value,ligne-taille,j,ordre_priorite)
        map = coef_priority(map,value,ligne+taille,j,ordre_priorite)

    for j in range(ligne-taille,ligne+taille+1,1):
        map = coef_priority(map,value,j,colonne+taille,ordre_priorite)
        map = coef_priority(map,value,j,colonne-taille,ordre_priorite)
    return map

def create_video(liste_img,nom,fps=30):
    # videodims = (liste_img[0].shape[0],liste_img[0].shape[1])
    videodims = liste_img[0].size
    print(videodims)
    fourcc = cv2.VideoWriter_fourcc(*'avc1')  
    # fourcc = cv2.VideoWriter_fourcc(*'MP4V')    
    print(fourcc)
    video = cv2.VideoWriter(f"{nom}.mp4",fourcc, fps,videodims)
    print(f"on sauvegarde la vidéo dans : {nom}")
    for i in liste_img:
        video.write(cv2.cvtColor(np.array(i), cv2.COLOR_RGB2BGR))
    video.release()

