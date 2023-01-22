# from fonctions import create_video
from fonction2 import *
from PIL import Image
import numpy as np
from time import time
from warnings import warn
import heapq
world = np.zeros((20,20))
# certainty_map = [-1]
# certainty_map = np.array([certainty_map*world.shape[0]*world.shape[1]]).reshape(world.shape)
position = [0,0]
global chemin_du_csvmur 
chemin_du_csvmur = "wall.csv"
# chemin_du_csvmur = "wallvide.csv"
global chemin_du_csvhum
chemin_du_csvhum = "leshumains2.csv"

what_to_display = "human"

valeur_human = 10
valeur_human_incertain = 9
valeur_mur = 4
mur_autour = 3
valeur_case_inconnue = -2
valeur_case_cool = -3
#allez savoir pourquoi, maintenant le loadtxt me fait chier car j'ai sauvegardé avec excel...
temp = open("wall2.csv")
wall = []
for i in temp:
    x = i.replace("\n",'').replace("ï»¿","").split(",")
    wall.append(x)
wall = np.array(wall).astype(float)
# wall = np.loadtxt("wall2.csv")
# wall = np.loadtxt("walltest.csv",delimiter=',')

nb_iterations = 0
liste_danger = [valeur_mur,mur_autour,valeur_human_incertain,valeur_human]


class Node:
    def __init__(self, parent=None, position=None):
        self.parent = parent
        self.position = position

        self.g = 0
        self.h = 0
        self.f = 0

    def __eq__(self, other):
        return self.position == other.position
    def __repr__(self):
      return f"{self.position} - g: {self.g} h: {self.h} f: {self.f}"
    def __lt__(self, other):
      return self.f < other.f
    def __gt__(self, other):
      return self.f > other.f

def return_path(current_node):
    path = []
    current = current_node
    while current is not None:
        path.append(current.position)
        current = current.parent
    return path[::-1]  # Return reversed path

def a_star(known_human,known_wall, start, end):
    maze = known_human.copy()
    
    wall = known_wall.copy()
    hum_pos = maze==valeur_human
    wall[hum_pos] = valeur_mur
    
    # if end==(7,6):
    #     print("oui")
    #     aaa = wall.copy()
    #     aaa[end[0],end[1]]=7
    #     aaa[start[0],start[1]]=7
    #     print(aaa.astype(int))
    wall[end[0],end[1]]=0
    wall = np.ndarray.tolist(wall)
    
    # Create start and end node
    start_node = Node(None, start)
    start_node.g = start_node.h = start_node.f = 0
    end_node = Node(None, end)
    end_node.g = end_node.h = end_node.f = 0
    
    open_list = []
    closed_list = []

    # Heapify the open_list and Add the start node
    heapq.heapify(open_list) 
    heapq.heappush(open_list, start_node)

    outer_iterations = 0
    max_iterations = (known_human.shape[0]*known_human.shape[1] // 2)
    max_iterations = 800

    adjacent_squares = ((0, -1), (0, 1), (-1, 0), (1, 0), (-1, -1), (-1, 1), (1, -1), (1, 1),)

    # Loop until you find the end
    while len(open_list) > 0:
        outer_iterations += 1

        if outer_iterations > max_iterations:
          # if we hit this point return the path such as it is
          # it will not contain the destination
          warn("giving up on pathfinding too many iterations")
          return return_path(current_node)       
        
        # Get the current node
        current_node = heapq.heappop(open_list)
        closed_list.append(current_node)

        # Found the goal
        if current_node == end_node:
            return return_path(current_node)

        # Generate children
        children = []
        
        for new_position in adjacent_squares: # Adjacent squares

            node_position = (current_node.position[0] + new_position[0], current_node.position[1] + new_position[1])
            if node_position[0] > (len(maze) - 1) or node_position[0] < 0 or node_position[1] > (len(maze[len(maze)-1]) -1) or node_position[1] < 0:
                continue

            # Make sure walkable terrain
            if wall[node_position[0]][node_position[1]] != 0:
            # if wall[node_position[0]][node_position[1]] != 0:
            # if known_human[node_position[0],node_position[1]] !=0 and known_wall[node_position[0],node_position[1]] !=0:
            # if known_human[node_position[0],node_position[1]] !=0 or known_wall[node_position[0],node_position[1]] !=0:
                continue

            # Create new node
            new_node = Node(current_node, node_position)

            # Append
            children.append(new_node)

        # Loop through children
        for child in children:
            # Child is on the closed list
            if len([closed_child for closed_child in closed_list if closed_child == child]) > 0:
                continue

            # Create the f, g, and h values
            child.g = current_node.g + 1
            child.h = ((child.position[0] - end_node.position[0]) ** 2) + ((child.position[1] - end_node.position[1]) ** 2)
            child.f = child.g + child.h

            # Child is already in the open list
            if len([open_node for open_node in open_list if child.position == open_node.position and child.g > open_node.g]) > 0:
                continue

            # Add the child to the open list
            heapq.heappush(open_list, child)

    warn("Couldn't get a path to destination")
    return None

def add_to_list(change_some_value=[False]):
    global liste_certainty_map_human
    global liste_certainty_map_wall
    global list_known_human
    global list_known_wall
    
    if change_some_value[0]:
        _,value,new_pos = change_some_value
        x = certainty_map_human.copy()
        pos_wall = known_wall==valeur_mur
        x[pos_wall] = valeur_mur
        x[new_pos[0],new_pos[1]]=value
        liste_certainty_map_human.append(x)
        
        x = certainty_map_wall.copy()
        pos_human = known_human==valeur_human
        x[pos_human] = valeur_human
        x[new_pos[0],new_pos[1]]=value
        liste_certainty_map_wall.append(x)

        x = known_human.copy()
        x[new_pos[0],new_pos[1]]=value
        list_known_human.append(x)

        x = known_wall.copy()
        x[new_pos[0],new_pos[1]]=value
        list_known_wall.append(x)
    else :
        x = certainty_map_human.copy()
        liste_certainty_map_human.append(x)
        
        x = certainty_map_wall.copy()
        liste_certainty_map_wall.append(x)

        x = known_human.copy()
        list_known_human.append(x)

        x = known_wall.copy()
        list_known_wall.append(x)

def not_the_border(position,world):
    lp,cp = position
    directions_possible = [
        1, #directions_possible[0] = up
        1, #directions_possible[1] = down
        1, #directions_possible[2] = left
        1, #directions_possible[3] = right
        1, #directions_possible[4] = topleft
        1, #directions_possible[5] = topright
        1, #directions_possible[6] = botleft
        1] #directions_possible[7] = botright

    if lp==0: #the bot is on the top edge of the map, therefore can't go up
        directions_possible[0] = 0
        directions_possible[4] = 0
        directions_possible[5] = 0
    elif lp==world.shape[0]: #the bot is on the bottom edge of the map, therefore can't go down
        directions_possible[1] = 0
        directions_possible[6] = 0
        directions_possible[7] = 0

    if cp==0: #the bot is on the left edge of the map, therefore can't go left
        directions_possible[2] = 0
        directions_possible[4] = 0
        directions_possible[6] = 0
    elif lp==world.shape[1]: #the bot is on the right edge of the map, therefore can't go right
        directions_possible[3] = 0
        directions_possible[5] = 0
        directions_possible[7] = 0
    return directions_possible

def new_position(position,direction):
    lp,cp = position
    if direction==0:
        lp +=-1
    elif direction==1:
        lp +=1
    elif direction==2:
        cp +=-1
    elif direction==3:
        cp +=1
    elif direction==4:
        lp +=-1
        cp +=-1
    elif direction==5:
        lp +=-1
        cp +=1
    elif direction==6:
        lp +=1
        cp +=-1
    else :
        lp +=1
        cp +=1
    return [lp,cp]

def check_ahead(position,certainty_map_human,certainty_map_wall,direction): #check if we already know the case 2 iterations in this directions
    new_pos = new_position(position,direction)
    lp,cp = new_pos
    if lp>=0 and lp<certainty_map_wall.shape[0] and cp>=0 and cp<certainty_map_wall.shape[1]:
        if certainty_map_human[new_pos[0],new_pos[1]] in liste_danger or certainty_map_wall[new_pos[0],new_pos[1]] in liste_danger:
            return False
            #pour la ligne qui suit, il faut vérifier mais je crois que "valeur case inconnue" n'est pas la bonne valeur, cependant flm pour le moment
        elif certainty_map_human[lp,cp]==-1 or certainty_map_human[lp,cp]==valeur_case_inconnue or certainty_map_wall[lp,cp]==-1 or certainty_map_wall[lp,cp]==valeur_case_inconnue:
            return True
        else:
            return False
    else:
        return False

def certainty_priority(map,value,ligne,colonne):
    ordre_priorite = [
        0,1.5,1,valeur_case_inconnue,-1
        ]
    if (ligne<map.shape[0]) and (ligne>=0) and (colonne<map.shape[1]) and (colonne>=0):
        if ordre_priorite.index(map[ligne,colonne])>ordre_priorite.index(value):
            map[ligne,colonne] = value
    return map 

def possible_wall_location_around(pos):
    global known_wall
    l,c = pos
    return known_wall[max(0,l-1):min(known_wall.shape[0],l+2),max(0,c-1):min(known_wall.shape[0],c+2)]

def possible_human_location_around(pos):
    global known_human
    l,c = pos
    known_human[l,c] = 5
    to_return = known_human[max(0,l-3):min(known_human.shape[0],l+4),max(0,c-3):min(known_human.shape[0],c+4)]
    
    return to_return

def guess_around(world,pos_bot,certainty_map_human,certainty_map_wall):

    # on charge les cartes de positions de mur/humain
    # elles servent à voir sur quel type de case se trouve le robot
    temp = open(chemin_du_csvmur)
    wall_map = []
    for i in temp:
        x = i.replace("\n",'').replace("ï»¿","").split(",")
        wall_map.append(x)
    wall_map = np.array(wall_map).astype(float) 
    # wall_map = np.loadtxt("walltest.csv",delimiter=',')
    global chemin_du_csvhum
    human_map = np.loadtxt(chemin_du_csvhum,delimiter=',')

    global known_wall
    global known_human
    ligne_bot,colonne_bot = pos_bot
    
    priority_human = [valeur_human,0,6,7,8,valeur_case_inconnue,valeur_case_cool,valeur_human_incertain,-1]
    priority_wall = [2,1.5,1,0.5,0,valeur_case_inconnue,valeur_mur,mur_autour,-1]
    #if we know there's a wall around (for sure), we do not want it to interfere with our logic
    position_interessante = [0,14]
        
    pwla = possible_wall_location_around(position)

    #here we get the value of the cell on which the bot is standing
    score_wall = wall_map[ligne_bot,colonne_bot]

    score_human= human_map[ligne_bot,colonne_bot]
    score = score_human + score_wall

    if score_wall ==0:
        known_wall[ligne_bot,colonne_bot] = 0
        known_wall = cercle_autourv2(known_wall,pos_bot,3,0,[valeur_mur,0,-1])

        certainty_map_wall = cercle_autourv2(certainty_map_wall,pos_bot,3,valeur_case_inconnue,priority_wall)
        certainty_map_wall[ligne_bot,colonne_bot] = 0
        
    if score_human ==0:
        known_human[ligne_bot,colonne_bot] = 0
        known_human = cercle_autourv2(known_human,pos_bot,3,0,[valeur_human,0,valeur_human_incertain,-1])
        known_human = cercle_autourv2(known_human,pos_bot,5,0,[valeur_human,0,valeur_human_incertain,-1])
        known_human = cercle_autourv2(known_human,pos_bot,7,0,[valeur_human,0,valeur_human_incertain,-1])

        certainty_map_human = cercle_autourv2(certainty_map_human,pos_bot,3,valeur_case_inconnue,priority_human)
        certainty_map_human = cercle_autourv2(certainty_map_human,pos_bot,5,valeur_case_inconnue,priority_human)
        certainty_map_human = cercle_autourv2(certainty_map_human,pos_bot,7,valeur_case_inconnue,priority_human)
        certainty_map_human[ligne_bot,colonne_bot] = 0

    if score_wall!=0:
        
        pwla = possible_wall_location_around(position)
        
        nb_new_wall = int(wall_map[ligne_bot,colonne_bot]/0.5) - np.sum(pwla==valeur_mur)
        
        if np.sum(pwla==-1)==nb_new_wall or nb_new_wall==0: 

            lp,cp = np.where(pwla==-1)
            l,c = position
            if nb_new_wall!=0:
                for i in range(len(lp)):
                    if l+lp[i]-1>=0 and l+lp[i]-1<known_wall.shape[0] and c+cp[i]-1>=0 and c+cp[i]-1<known_wall.shape[1]:
                        known_wall[l+lp[i]-1,c+cp[i]-1] = valeur_mur
                        certainty_map_wall[l+lp[i]-1,c+cp[i]-1] = valeur_mur
                        # print("on a bien ajouté le mur")
            else:
                for i in range(len(lp)):
                    if l+lp[i]-1>=0 and l+lp[i]-1<known_wall.shape[0] and c+cp[i]-1>=0 and c+cp[i]-1<known_wall.shape[1]:
                        known_wall[l+lp[i]-1,c+cp[i]-1] = 0
                        certainty_map_wall[l+lp[i]-1,c+cp[i]-1] = valeur_case_inconnue
                        # print("on sait déjà où sont les murs, ces cases sont sans mur")
                certainty_map_wall[ligne_bot,colonne_bot] = score_wall
        else :
            certainty_map_wall = cercle_autourv2(certainty_map_wall,pos_bot,3,mur_autour,[0,valeur_case_inconnue,valeur_mur,0.5,1,1.5,2,mur_autour,-1])
        certainty_map_wall[ligne_bot,colonne_bot] = score_wall
        known_wall[ligne_bot,colonne_bot] = 0

    if score_human !=0:
        
        
        if score_human==8:
            # humain à 1 cases du robots 
            certainty_map_human = cercle_autourv2(certainty_map_human,pos_bot,3,valeur_human_incertain,priority_human)     
            certainty_map_human[ligne_bot,colonne_bot] = 0
        elif score_human==7:
            # if pos_bot==[11,8] or pos_bot==(11,8):
            #     print("\n\n\nici\n\n\n")
            # humain à 2 cases du robots 
            certainty_map_human = cercle_autourv2(certainty_map_human,pos_bot,3,valeur_case_inconnue,priority_human)
            certainty_map_human = cercle_autourv2(certainty_map_human,pos_bot,5,valeur_human_incertain,priority_human)  
            
            known_human = cercle_autourv2(known_human,pos_bot,3,0,priority_human)        
            certainty_map_human[ligne_bot,colonne_bot] = 0
        elif score_human==6:
            # humain à 3 cases du robots 
            certainty_map_human = cercle_autourv2(certainty_map_human,pos_bot,3,valeur_case_inconnue,priority_human)
            certainty_map_human = cercle_autourv2(certainty_map_human,pos_bot,5,valeur_case_inconnue,priority_human)
            certainty_map_human = cercle_autourv2(certainty_map_human,pos_bot,7,valeur_human_incertain,priority_human)     

            known_human = cercle_autourv2(known_human,pos_bot,3,0,priority_human)
            known_human = cercle_autourv2(known_human,pos_bot,5,0,priority_human)
            certainty_map_human[ligne_bot,colonne_bot] = 0

        phla = possible_human_location_around(position)

        if np.sum(phla==-1)==1 and np.sum(phla==valeur_human)==0 and np.sum(phla==valeur_human_incertain)==0:
            
            
            lp,cp = np.where(phla==-1)
            lb,cb = np.where(phla==5)
            lp,cp = lp[0],cp[0]
            l,c = position    
            
            if l+lp-lb>=0 and l+lp-lb<known_human.shape[0] and c+cp-cb>=0 and c+cp-cb<known_human.shape[1]:
                print(phla)
                known_human[l+lp-lb,c+cp-cb] = valeur_human
                certainty_map_human[l+lp-lb,c+cp-cb] = valeur_human
                print(f"humain posé sur {l+lp-lb},{c+cp-cb} : ({l}+{lp}-{lb}),({c}+{cp}-{cb})")
        

        
        known_human[ligne_bot,colonne_bot] = 0


    all_cool_cell = np.where(certainty_map_human==valeur_case_cool)
    for i in range(len(all_cool_cell[0])):
        surroundings = certainty_map_human[max(0,all_cool_cell[0][i]-1):min(known_wall.shape[0],all_cool_cell[0][i]+2),max(0,all_cool_cell[1][i]-1):min(known_wall.shape[0],all_cool_cell[1][i]+2)]
        if np.sum(surroundings==valeur_human_incertain)==0:
            # if [all_unknown_cell[0][i],all_unknown_cell[1][i]]==[9,6] or [all_unknown_cell[0][i],all_unknown_cell[1][i]]==[8,6] or [all_unknown_cell[0][i],all_unknown_cell[1][i]]==[7,6]:
            #     print(f"\n\n pour {[all_unknown_cell[0][i],all_unknown_cell[1][i]]}:\n{surroundings}")
            certainty_map_human[all_cool_cell[0][i],all_cool_cell[1][i]] = valeur_case_inconnue


    # certainty_map[ligne_bot,colonne_bot] = score_wall
    all_unknown_cell = np.where(certainty_map_human==-2) 
    for i in range(len(all_unknown_cell[0])):
        surroundings = certainty_map_human[max(0,all_unknown_cell[0][i]-1):min(known_wall.shape[0],all_unknown_cell[0][i]+2),max(0,all_unknown_cell[1][i]-1):min(known_wall.shape[0],all_unknown_cell[1][i]+2)]
        if np.sum(surroundings==valeur_human_incertain)!=0:
            # if [all_unknown_cell[0][i],all_unknown_cell[1][i]]==[9,6] or [all_unknown_cell[0][i],all_unknown_cell[1][i]]==[8,6] or [all_unknown_cell[0][i],all_unknown_cell[1][i]]==[7,6]:
            #     print(f"\n\n pour {[all_unknown_cell[0][i],all_unknown_cell[1][i]]}:\n{surroundings}")
            certainty_map_human[all_unknown_cell[0][i],all_unknown_cell[1][i]] = valeur_case_cool
        else: 
            certainty_map_human[all_unknown_cell[0][i],all_unknown_cell[1][i]] = valeur_case_inconnue

    certainty_map_human[ligne_bot,colonne_bot] = human_map[ligne_bot,colonne_bot]
    certainty_map_wall[ligne_bot,colonne_bot] = wall_map[ligne_bot,colonne_bot]
    
    return certainty_map_human, certainty_map_wall

def diag_first(position, target, certainty_map):
    ligp,colp = position 
    sign_lig = 1 if ligp-target[0]<0 else -1
    sign_col = 1 if colp-target[1]<0 else -1
    list_position = []
    stop = False
    Failed = False
    while (ligp!=target[0] and colp!=target[1]) and not stop:
        ligp += sign_lig
        colp += sign_col
        if certainty_map[ligp,colp] in liste_danger :#mur ou humain ou jsp
            stop = True
            Failed = True
        else :
            list_position.append([ligp,colp])

    while ligp!=target[0] and not stop:
        ligp += sign_lig
        if certainty_map[ligp,colp] in liste_danger :#mur ou humain ou jsp
            stop = True
            Failed = True
        else :
            list_position.append([ligp,colp])
    while colp!=target[1] and not stop:
        colp += sign_col
        if certainty_map[ligp,colp] in liste_danger :#mur ou humain ou jsp
            stop = True
            Failed = True
        else :
            list_position.append([ligp,colp])

    return list_position,Failed
 
def diag_second(position, target, certainty_map):
    ligp,colp = position 
    sign_lig = 1 if ligp-target[0]<0 else -1
    sign_col = 1 if colp-target[1]<0 else -1
    list_position = []
    stop = False
    Failed = False
    while (abs(abs(ligp)-abs(target[0]))!= abs(abs(colp)-abs(target[1]))) and not stop:
        if abs(abs(ligp)-abs(target[0]))>abs(abs(colp)-abs(target[1])):
            ligp += sign_lig
        else :
            colp += sign_col
        # print(f"ligne : target={target[0]}, ligne actuelle {ligp}, dif abs={abs(ligp)-abs(target[0])}")
        # print(f"colonne : target={target[1]}, ligne actuelle {colp}, dif abs={abs(colp)-abs(target[1])}")
        # print("\n")
        if certainty_map[ligp,colp] in liste_danger :#mur ou humain ou jsp
            stop = True
            Failed = True
        else :
            list_position.append([ligp,colp])

    while (ligp!=target[0] and colp!=target[1]) and not stop:
        ligp += sign_lig
        colp += sign_col
        if certainty_map[ligp,colp] in liste_danger :#mur ou humain ou jsp
            stop = True
            Failed = True
        else :
            list_position.append([ligp,colp])

    return list_position,Failed

def turn_first(position, target, certainty_map):
    ligp,colp = position 
    sign_lig = 1 if ligp-target[0]<0 else -1
    sign_col = 1 if colp-target[1]<0 else -1
    list_position = []
    stop = False
    Failed = False
    while ligp!=target[0] and not stop:
        ligp+=sign_lig
        if certainty_map[ligp,colp]in liste_danger  :#mur ou humain ou jsp
            stop = True
            Failed = True
        else :
            list_position.append([ligp,colp])

    while colp!=target[1] and not stop:
        colp+=sign_col
        if certainty_map[ligp,colp] in liste_danger :#mur ou humain ou jsp
            stop = True
            Failed = True
        else :
            list_position.append([ligp,colp])
    return list_position,Failed

def turn_second(position, target, certainty_map):
    ligp,colp = position 
    sign_lig = 1 if ligp-target[0]<0 else -1
    sign_col = 1 if colp-target[1]<0 else -1
    list_position = []
    stop = False
    Failed = False
    while colp!=target[1] and not stop:
        colp+=sign_col
        if certainty_map[ligp,colp] in liste_danger  :#mur ou humain ou jsp
            stop = True
            Failed = True
        else :
            list_position.append([ligp,colp])
    while ligp!=target[0] and not stop:
        ligp+=sign_lig
        if certainty_map[ligp,colp] in liste_danger  :#mur ou humain ou jsp
            stop = True
            Failed = True
        else :
            list_position.append([ligp,colp])

    return list_position,Failed

def got_to_point(position,target,certainty_map_human,certainty_map_wall):

    did_wall_fail = True
    did_human_fail = True
    # if position ==[1,7]:
    #     print("\n pour le pathfinding: vers",target)
    path, did_human_fail = diag_first(position,target,certainty_map_human)
    path, did_wall_fail = diag_first(position,target,certainty_map_wall)
    # if position ==[1,7]:
    #     print("1:",did_human_fail,did_wall_fail)

    if did_human_fail or did_wall_fail: # if the previous one fails, we try the next one, otherwise we just go
        path, did_human_fail = diag_second(position,target,certainty_map_human)
        path, did_wall_fail  = diag_second(position,target,certainty_map_wall)
    # if position ==[1,7]:
    #     print("2",did_human_fail,did_wall_fail)
    if did_human_fail or did_wall_fail:
        path, did_human_fail = turn_first(position,target,certainty_map_human)
        path, did_wall_fail  = turn_first(position,target,certainty_map_wall)
    # if position ==[1,7]:
    #     print("3",did_human_fail,did_wall_fail)
    if did_human_fail or did_wall_fail:
        path, did_human_fail = turn_second(position,target,certainty_map_human)
        path, did_wall_fail  = turn_second(position,target,certainty_map_wall)
    # if position ==[1,7]:
    #     print("4",did_human_fail,did_wall_fail)
    
    # print(f"path from {position} to {target} : {path}")
    if did_human_fail or did_wall_fail:
        # print(f"we couldn't get from {position} to the point {target} ")
        #it has to return False because it failed, but one of the two may have succeeded so we can just return any of them (talking about did_x_fail)
        if len(path)>1:
            return path, False
        else:
            return path, False
    else:
        if len(path)>1:
            return path, True
        else:
            return path, True

def go_from_somewhere_else(position,target,certainty_map_human,certainty_map_wall):
    #algorithme de gitan qui va être utilisé si on a pas pu passer atteindre une destination depuis la position initiale
    #l'idée étant que peut-être qu'en se déplaçant sur une case voisine ça passe 🤷‍♀️
    possible_direction = not_the_border(position,certainty_map_human) 
    sauvetage = False
    global liste_certainty_map_human
    global liste_certainty_map_wall
    global list_known_human
    global list_known_wall
    global path_done
    global known_wall
    # print("la target",target)
    for i in range(len(possible_direction)):
        if possible_direction[i]==1:
            new_pos = new_position(position,i)
            lp,cp = new_pos
            if certainty_map_human[new_pos[0],new_pos[1]] not in liste_danger and certainty_map_wall[new_pos[0],new_pos[1]] not in liste_danger:
                add_to_list(change_some_value=[True,50,new_pos])

                path_done.append(position)
                path, sauvetage = got_to_point(new_pos,target,certainty_map_human,certainty_map_wall)
            if sauvetage:
                print("on est sauvé")
                path = [new_pos] + path
                print("on part de ",position)
                print("et on fait",path)
                return path,sauvetage
    return [],False

def update_uncertain_wall(certainty_map,position):
    l,c = position
    surroundings = certainty_map[max(0,l-1):min(certainty_map.shape[0],l+2),max(0,c-1):min(certainty_map.shape[0],c+2)]
    if np.sum(surroundings==mur_autour)==0:
        return certainty_map #on return sans changement si on n'a aucune case avec un possible mur à coté
    else:
        value_case = certainty_map[l,c]
        known_surrounding_walls = possible_wall_location_around(position)
        nb_wall_found_around = np.sum(known_surrounding_walls==valeur_mur)
        nb_unknown_case_around = np.sum(surroundings==valeur_case_inconnue)
        nb_actual_wall_around = value_case/0.5
        if nb_wall_found_around==nb_actual_wall_around:
            lp,cp = np.where(surroundings==mur_autour)
            for i in range(len(lp)):
                if l+lp[i]-1>=0 and l+lp[i]-1<known_wall.shape[0] and c+cp[i]-1>=0 and c+cp[i]-1<known_wall.shape[1]:
                    known_wall[l+lp[i]-1,c+cp[i]-1] = 0
                    certainty_map[l+lp[i]-1,c+cp[i]-1] = valeur_case_inconnue
                    # print(f"on update l'affichage en case inconnue/non mur {[l+lp[i]-1,c+cp[i]-1]}")

        elif nb_wall_found_around +np.sum(surroundings==mur_autour) ==nb_actual_wall_around:
            lp,cp = np.where(surroundings==mur_autour)
            for i in range(len(lp)):
                if l+lp[i]-1>=0 and l+lp[i]-1<known_wall.shape[0] and c+cp[i]-1>=0 and c+cp[i]-1<known_wall.shape[1]:
                    known_wall[l+lp[i]-1,c+cp[i]-1] = valeur_mur
                    certainty_map[l+lp[i]-1,c+cp[i]-1] = valeur_mur
                    # print(f"on update l'affichage en mur en {[l+lp[i]-1,c+cp[i]-1]}")
    return certainty_map
    
def make_deduction(certainty_map):
    #the idea of this function is :
    #we check around each case "x wall around" if there are "maybe wall" cases around
    #because of how our code works, theses case are not automatically updated
    #so we will update them if by deduction of the surrounding cases can help us
    #ex: if we find a "1 wall around" case, and we found the said wall, we update the "maybe wall" case
    range_mur = [2,1.5,1,0.5]

    all_coord_wall_around = []
    global known_wall
    nb_found_wall = 0
    
    while nb_found_wall!= np.sum(known_wall==4):
        nb_found_wall = np.sum(known_wall==4)
        for x in range_mur:
            cases =np.where(certainty_map==x)
            lp,cp = cases
            for i in range(len(lp)):
                all_coord_wall_around.append([lp[i],cp[i]])
        
        for x in all_coord_wall_around:
            certainty_map= update_uncertain_wall(certainty_map,x)

def make_deduction_human():
    global certainty_map_human
    global known_human
    A = np.where(certainty_map_human==8)

    coords = []
    for i in range(len(A[0])):
        coords.append([A[0][i],A[1][i]])
    
    # print(coords)
    for point in coords:
        unknown_surroundings = certainty_map_human[max(0,point[0]-1):min(known_wall.shape[0],point[0]+2),max(0,point[1]-1):min(known_wall.shape[0],point[1]+2)]
        humans_around = np.sum(unknown_surroundings==valeur_human)
        # unknown_around= np.sum(surroundings==-1)
        known_surroundings = known_human[max(0,point[0]-1):min(known_wall.shape[0],point[0]+2),max(0,point[1]-1):min(known_wall.shape[0],point[1]+2)]
        uncertn_around= np.sum(known_surroundings==-1)
        
        # if point ==[19,13] or point==[18,13]:
        #     print("point : ",point,f"humans_around : {humans_around}, uncertn around : {uncertn_around} surroundings : \n",known_surroundings,"\n",unknown_surroundings)
    
        if humans_around==0 and uncertn_around==1:
            actual_human = np.where(known_surroundings==-1)
            if point[0]==0:
                rel_l = 0
            else :
                rel_l = -1
            if point[1]==0:
                rel_c = 0
            else :
                rel_c = -1
            acutal_coord = [point[0]+rel_l+actual_human[0][0],point[1]+rel_c+actual_human[1][0]]
            print(f"en la case {point}, la case en {acutal_coord} est un humain ! au lieu de simplement valoir {certainty_map_human[acutal_coord[0],acutal_coord[1]]}, regarde par toi-même \n{unknown_surroundings}")
            certainty_map_human[acutal_coord[0],acutal_coord[1]]=valeur_human
            known_human[acutal_coord[0],acutal_coord[1]]=valeur_human

def case_dangereuse(position,certainty_map_human,certainty_map_wall):
    c,l = position
    if certainty_map_human[c,l] not in [valeur_human,valeur_human_incertain] and certainty_map_wall[c,l] not in [mur_autour,valeur_mur]:
        return False
    else :
        return True

def find_path_to_case(map,position,taille,value_to_find,debug=False):
    global certainty_map_human
    global certainty_map_wall

    list_point_autour_pre_verif =cherche_autourv2(position,map,certainty_map_human,certainty_map_wall,value_to_find=value_to_find,taille=taille)
    list_point_autour = []
    if len(list_point_autour_pre_verif)!=1:
        for point in list_point_autour_pre_verif[:-1]:
            if certainty_map_human[point[0],point[1]] not in [valeur_human,valeur_human_incertain] and certainty_map_wall[point[0],point[1]] not in [valeur_mur,mur_autour]:
                list_point_autour.append(point)
    else:
        list_point_autour = list_point_autour_pre_verif
    chemin= []
    succes = False
    # if position==[14,7] and debug:
    #     print(f"\n\npour [14,7] avec taille = {taille}:")
    #     print(list_point_autour)
    #     print(list_point_autour_pre_verif)
    if len(list_point_autour)!=0:
        
        i=-1
        no_path_found = False
        # print(f"i= {nb_iterations} entre dans while taille {taille} depuis {position} vers", list_point_autour)
        while not succes and not no_path_found:
            i+=1
            if i<len(list_point_autour):
                chemin,succes = got_to_point(position,list_point_autour[i],certainty_map_human,certainty_map_wall)
            else :
                no_path_found = True
    return chemin,succes

def liste_points_isoles():
    global known_human
    global certainty_map_human
    all_unknown = np.where(known_human==-1)
    points_isoles = []
    for i in range(len(all_unknown[0])):
        neighbour = known_wall[max(0,all_unknown[0][i]-1):min(known_wall.shape[0],all_unknown[0][i]+2),max(0,all_unknown[1][i]-1):min(known_wall.shape[0],all_unknown[1][i]+2)]

        if np.sum(neighbour==-1)<2:
            c_neighbour =certainty_map_human[max(0,all_unknown[0][i]-1):min(known_wall.shape[0],all_unknown[0][i]+2),max(0,all_unknown[1][i]-1):min(known_wall.shape[0],all_unknown[1][i]+2)].copy()
            points_isoles.append(([all_unknown[0][i],all_unknown[1][i]],neighbour.copy(),c_neighbour))
    return points_isoles

human = np.zeros((20,20))
start_position = [0,17]
# start_position = [18,1]
position = start_position
direction = 0


global known_wall
known_wall = [-1]
known_wall = np.array([known_wall*world.shape[0]*world.shape[1]],dtype=float).reshape(world.shape)
global known_human
known_human = [-1]
known_human = np.array([known_human*world.shape[0]*world.shape[1]],dtype=float).reshape(world.shape)



global certainty_map_human
global certainty_map_wall
certainty_map_human = [-1]
certainty_map_human = np.array([certainty_map_human*world.shape[0]*world.shape[1]],dtype=float).reshape(world.shape)
certainty_map_wall = [-1]
certainty_map_wall = np.array([certainty_map_wall*world.shape[0]*world.shape[1]],dtype=float).reshape(world.shape)


global path_done
path_done = [position]

global liste_certainty_map_human
global liste_certainty_map_wall
global list_known_human
global list_known_wall

list_known_human = [known_human.copy()]
list_known_wall = [known_wall.copy()]
liste_certainty_map_wall = [certainty_map_wall.copy()]
liste_certainty_map_human = [certainty_map_human.copy()]
certainty_map_human,certainty_map_wall = guess_around(world,position,certainty_map_human,certainty_map_wall)

liste_certainty_map_wall.append(certainty_map_wall.copy())
liste_certainty_map_human.append(certainty_map_human.copy())
list_known_human.append(known_human.copy())
list_known_wall.append(known_wall.copy())

nb_iterations = 0
failed = False
# while np.sum(cert
# ainty_map==-1)!=0 and not(nb_iterations>450) and not failed:
pas_plus_loin = 300
already_targeted = []
while np.sum(known_human==10)!=9 and nb_iterations<pas_plus_loin:
    line_bot,col_bot = position
    make_deduction(certainty_map_wall)
    certainty_map_human,certainty_map_wall = guess_around(world,position,certainty_map_human,certainty_map_wall)


    if check_ahead(position,certainty_map_human,certainty_map_wall,direction) and nb_iterations<pas_plus_loin:
        position = new_position(position,direction)
        certainty_map_human,certainty_map_wall = guess_around(world,position,certainty_map_human,certainty_map_wall)
        path_done.append(position)

        add_to_list(change_some_value=[True,5,position])
        nb_iterations+=1
        #we did not explore what's ahead, and we know there isn't any obstacle, so we can continue on this direction

    elif nb_iterations<pas_plus_loin:
        #we have already been on this place, or it's the border, let's find another way
        certainty_map_wall = update_uncertain_wall(certainty_map_wall,position)
        succes = False
        failed = False
        taille = -1
        print(f"\ndébut position it={nb_iterations} : ",position)
        decallage = 7
        while not succes and taille<7:
            taille+=1
            chemin,succes = find_path_to_case(known_human,position,taille,-1)
            # print("taille : ", taille)
            # if succes :
            #     print("succès1 : ",succes)
            if not succes:
                chemin,succes = find_path_to_case(certainty_map_human,position,taille,valeur_case_cool)

            if not succes:
                chemin,succes = find_path_to_case(known_wall,position,taille,-1)
                # if succes:
                #     print("succès2 : ",succes)
            if taille>decallage:
                    # if succes:
                    #     print("succès3 : ",succes)
                if not succes:
                    chemin,succes = find_path_to_case(known_wall,position,taille-decallage,mur_autour)
                    # if succes:
                    #     print("succès4 : ",succes)
                if not succes:
                    chemin,succes = find_path_to_case(certainty_map_human,position,taille-decallage,-2)
                    # if succes:
                    #     print("succès5 : ",succes)
                if not succes:
                    chemin,succes = find_path_to_case(certainty_map_wall,position,taille-decallage,-2,debug=True)
                    # if succes:
                    #     print("succès6 : ",succes)


        # print(f"le chemin qu'on prendra depuis {position} : ",chemin)
        
        if succes:
            # print("on rentre dans succes")
            for chemin_a_prendre in chemin:
                if not case_dangereuse(chemin_a_prendre,certainty_map_human,certainty_map_wall):
                    position = chemin_a_prendre
                    certainty_map_human,certainty_map_wall = guess_around(world,position,certainty_map_human,certainty_map_wall)
                    path_done.append(position)
                    
                    add_to_list(change_some_value=[True,5,position])

                    nb_iterations+=1
                else : 
                    break
            possible_direction = not_the_border(position,world) 
            i=0
            while possible_direction[i]!=1:
                i+=1
            direction = possible_direction[i]
        else:
            # print("position quand on a échoué : ",position)
            case = []
            taille=0
            valid_cases=[]

            # while len(case)==0 and taille<20:
            while taille<20:
                taille+=1
                if np.sum(certainty_map_human==valeur_case_cool)==0:
                    case += cherche_autourv2(position,known_human,certainty_map_human,certainty_map_wall,value_to_find=-1,taille=taille,forastar=True)
                else: 
                    case += cherche_autourv2(position,certainty_map_human,certainty_map_human,certainty_map_wall,value_to_find=valeur_case_cool,taille=taille,forastar=True)
                
            path = [[-1,-1]]
            tar_astar =[-5,5]
            i = 0
            # print("on veut 13,10",case)
            while path[-1]!=tar_astar and i<len(case):
                
                try :
                    pos_astar = (position[0],position[1])
                    tar_astar = (case[i][0],case[i][1])
                    path = a_star(known_human,known_wall,pos_astar,tar_astar)[1:]
                except:
                    path = [[-1,-1]]
                # print(f"conséquences de {path[0]}: {certainty_map_wall[path[0][0],path[0][1]]} et {certainty_map_wall[path[0][0],path[0][1]]}")
                if certainty_map_wall[path[0][0],path[0][1]] in [mur_autour,valeur_mur] or certainty_map_human[path[0][0],path[0][1]] in [valeur_human_incertain,valeur_human]:
                    path = [[-1,-1]]
                i+=1
            if i>=len(case):
                if len(case)>0:
                    target = case[0]
                    case2 = []
                    taille=0
                    while taille<10:
                        taille+=1   
                        if np.sum(certainty_map_human==valeur_case_cool)==0:
                            case2 += cherche_autourv2(target,known_human,certainty_map_human,certainty_map_wall,value_to_find=-1,taille=taille,forastar=True)
                        else: 
                            case2 += cherche_autourv2(target,certainty_map_human,certainty_map_human,certainty_map_wall,value_to_find=valeur_case_inconnue,taille=taille,forastar=True)
                    
                    path = [[-1,-1]]
                    tar_astar =[-5,5]
                    i=0
                    while path[-1]!=tar_astar and i<len(case2):
                        try :
                            # print(" a coté de  ",position," il y a :",case2[i])
                            pos_astar = (position[0],position[1])
                            tar_astar = (case2[i][0],case2[i][1])
                            
                            # print(f"on prendra pos ={pos_astar} et tar={tar_astar} ")
                            path = a_star(known_human,known_wall,pos_astar,tar_astar)
                            # print(path)
                            # print(path[-1],tar_astar)
                            # if path[-1]!=tar_astar :
                            #     print("\n\nmais wtf c'est quoi cet algorithme ?",path[-1],tar_astar)
                        except:
                            # print("échoué")            
                            path = [[-1,-1]]
                            # break
                        i+=1
                else:
                    # print("dommage, on peut vraiment rien faire askip",np.sum(certainty_map_human==valeur_case_cool),np.sum(known_human==10),np.sum(known_human==-1))
                    # print(position)
                    break
            if i>=len(case):
                # print("dommage, on peut vraiment rien faire askip",np.sum(certainty_map_human==valeur_case_cool),np.sum(known_human==10),np.sum(known_human==-1))
                # print(position)
                # print(case)
                pass
            position_avant = position
            for chemin_a_prendre in path:

                if not case_dangereuse(chemin_a_prendre,certainty_map_human,certainty_map_wall):
                    position = chemin_a_prendre
                    certainty_map_human,certainty_map_wall = guess_around(world,position,certainty_map_human,certainty_map_wall)
                    path_done.append(position)
                    add_to_list(change_some_value=[True,15,position])
                    nb_iterations+=1
                else : 
                    break
            if position == position_avant:
                hum_around_position = certainty_map_human[max(0,position[0]-1):min(known_wall.shape[0],position[0]+2),max(0,position[1]-1):min(known_wall.shape[0],position[1]+2)]==valeur_case_inconnue
                wall_around_position = certainty_map_wall[max(0,position[0]-1):min(known_wall.shape[0],position[0]+2),max(0,position[1]-1):min(known_wall.shape[0],position[1]+2)]==valeur_case_inconnue
                safe_mouv = wall_around_position*hum_around_position
                if np.sum(safe_mouv)!=0:
                    rel_safe_pos = np.where(safe_mouv)
                    if position[0]==0:
                        rel_l = 0
                    else :
                        rel_l = -1
                    if position[1]==0:
                        rel_c = 0
                    else :
                        rel_c = -1
                    safe_pos = [position[0]+rel_l+rel_safe_pos[0][0],position[1]+rel_c+rel_safe_pos[1][0]]
                    print(f"on est resté immobile, on passe de {position} vers {safe_pos} car c'est safe, regarde par toi-même \n{safe_mouv}")
                    
                    position = safe_pos
                    certainty_map_human,certainty_map_wall = guess_around(world,position,certainty_map_human,certainty_map_wall)
                    path_done.append(position)
                    add_to_list(change_some_value=[True,15,position])
                    nb_iterations+=1

                # if np.sum(safe_mouv)!=0:
                #     rel_safe_pos = np.where(safe_mouv)
                #     if position[0]==0:
                #         rel_l = 0
                #     else :
                #         rel_l = -1
                #     if position[1]==0:
                #         rel_c = 0
                #     else :
                #         rel_c = -1
                #     safe_pos = [position[0]+rel_l+rel_safe_pos[0][0],position[1]+rel_c+rel_safe_pos[1][0]]
                #     print(f"on est resté immobile, on passe de {position} vers {safe_pos} car c'est safe, regarde par toi-même \n{safe_mouv}")
                    
                #     position = safe_pos
                #     certainty_map_human,certainty_map_wall = guess_around(world,position,certainty_map_human,certainty_map_wall)
                #     path_done.append(position)
                #     add_to_list(change_some_value=[True,15,position])
                #     nb_iterations+=1

    # if np.sum(known_human==10)==8:             
    #     make_deduction_human()
    make_deduction_human()
make_deduction_human()
add_to_list()
# print(np.sum(known_human==10)!=9)
print(certainty_map_human.astype(int))
list_to_display = []
list_to_display_debug = []
font                   = cv2.FONT_HERSHEY_SIMPLEX
bottomLeftCornerOfText = (10,500)
fontScale              = 2
fontColor              = (0,0,0)
# fontColor              = (255,255,255)
thickness              = 2
lineType               = 2
alphabet = "abcdefghijklmnopqrstuwxyz"
print(len(liste_certainty_map_human),len(list_known_human),len(liste_certainty_map_wall),len(list_known_wall))
if len(liste_certainty_map_human) == len(list_known_human):
    if len(liste_certainty_map_human) == len(liste_certainty_map_wall):
        if len(liste_certainty_map_human) == len(list_known_wall):
            for j in range(min(len(liste_certainty_map_human),500)):
                # img = display_single_map(j,dico_couleurs)
                
                img1 = display_nice_image(list_known_human[j],liste_certainty_map_human[j],dico_human,dico_couleurs)
                img2 = display_nice_image(list_known_wall[j],liste_certainty_map_wall[j],dico_mur,dico_couleurs)
                try :
                    #on essaye de récupérer les coordonnées du robot pour les écrire sur la carte, c'est juste ça
                    img1 = np.asarray(img1)
                    x =np.where(liste_certainty_map_human[j]==5)
                    cv2.putText(img1,f"{x[0][0]} {x[1][0]} ({alphabet[x[1][0]]})", 
                                bottomLeftCornerOfText, 
                                font, 
                                fontScale,
                                fontColor,
                                thickness,
                                lineType)
                    img1 = Image.fromarray(img1)
                except:
                    # img = display_nice_image(list_known_wall[j],liste_certainty_map[j],dico_mur,dico_couleurs)
                    # img = display_nice_image(list_known_wall[j],liste_certainty_map[j],dico_mur,dico_couleurs)
                    pass
                img = Image.fromarray(np.concatenate((np.asarray(img1),np.asarray(img2)),axis=0))
                list_to_display.append(img)
                list_to_display_debug.append(img1)
            
            create_video(list_to_display,f"cartes/carte_explore{start_position[0]}_{start_position[1]}", fps=min(int(nb_iterations/10),45))
            # create_video(list_to_display_debug,f"cartes/carte_explore{start_position[0]}_{start_position[1]}", fps=min(int(nb_iterations/10),45))

# create_video(list_to_display,"carte_explore", fps=min(int(nb_iterations/10),5))
print("vidéo crée : ",f"carte_explore{start_position[0]}_{start_position[1]}.mp4")
print("nombre d'iteration : ",nb_iterations)
# print(len(liste_certainty_map))
# print(liste_certainty_map[0]==certainty_map)
