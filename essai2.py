# -*- coding:Utf8 -*-
#########################################################################################
#
#     TP IA Resolution de Taquins
#       Master 1 IRCOMS 2018-2019
#       Chloe Delahaye, Nathan Teboul
#
#########################################################################################
import time
import sys
from copy import deepcopy
from heapq import heapify, heappush, heappop

#########################################################################################
#
#     CLASSE TAQUIN
#
#########################################################################################

class Taquin:
    
    # Initialisation du Taquin
    # size: taille du taquin (ici 3 ou 4)
    # taquin: initialisation de la grille
    # parent : liste des mouvements du trou pour parvenir á l'état courant.
    # parent
    # g : cout du chemin actuel. 
    # f : fonction d'évaluation pour A*. 
    # h: valeur trouvée grace à l'heuristique
    # goal: taquin resolu
    def __init__(self, n):
        self.size = n
        self.taquin = [[0] * n for _ in range(n)]
        self.f = 0
        self.parent = None        
        self.g = 0
        self.h = 0
        if n ==3:
            self.goal = [[0,1,2],[3,4,5],[6,7,8]]
        if n ==4:
            self.goal = [[0,1,2,3],[4,5,6,7],[8,9,10,11],[12,13,14,15]]
    
    # Genere une grille de taquin aléatoire
    # n taille du taquin
    def generate_grid(self,n):
        node=[i for i in range(n*n)]
        for i in range(self.size):
            for j in range(self.size):
                from random import randint
                value_index=randint(0, len(node)-1)
                self.taquin[i][j] = node[value_index]
                del node[value_index]

              
       
    #Affichage de la grille
    def print_grid(self):
        print(self.taquin)
    
    # Compte le nombre de permutation de la grille initiale 
    # tableau permettant de compter les permutations
    # n: taille du taquin
    def getInvCount(self,arr,n): 
        inv_count = 0
        for i in range((n*n)-1): 
            for j in range (i+1,(n*n)):
                 
                if (arr[i] and arr[j] and arr[i] > arr[j]):
                    inv_count=inv_count+1

            
        return inv_count
    
    # Dertermine la position du zero dans la grille initiale par rapport à dans la grille finale
    # n : taille du taquin
    def findPosition(self,n):
        for i in range(n-1,-1,-1):
            for j in range(n-1,-1,-1):
                if self.taquin[i][j]==0:
                    return n-i


    # Vérification de si la résolution du Taquin est possible
    # pour cela il faut :
    # si la taille est impair : il faut que le nombre de permutation soit pair
    # si la taille est paire il faut : que le nombre de permutation soit paire et la distance actuelle du zero à sa bonne position soit impaire
    # ou l'inverse
    #n : taille du taquin
    def isSolvable(self,n):
        arr=[]
        for i in range(n):
            for j in range(n):
                arr.append(self.taquin[i][j])
        

        invcount=self.getInvCount(arr,n)
        if n % 2 ==1:
            return(invcount%2==0)
                                
        else:
            pos = self.findPosition(n)
            if pos % 2 ==1:
                return (invcount%2==0)
            else : 
                return (invcount%2==1)

    # retroune vrai qi le taquin est resolu
    def is_solution(self):
        return self.taquin == self.goal

    # Calcul de la distance de Manhattan
    # n : taille du taquin
    def manhattan(self,n):
        h = 0
        for i in range(n):
            for j in range(n):
                x, y = divmod(self.taquin[i][j], n)
                h += abs(x-i) + abs(y-j)
        return h

    # other: autre taquin
    def __eq__(self, other):
        return self.taquin == other.taquin

    # Calcul du nombre de cases mal placées
    # n : taille du taquin
    def misplaced_Tiles(self, n):
        misplaced_tiles=0
        for i in range(n):
            for j in range(n):
                if(self.taquin[i][j]!=self.goal[i][j]):
                    misplaced_tiles+=1
        return misplaced_tiles


       
#########################################################################################
#
#     Algo A*
#
#########################################################################################

# fonction de déplacement de case
# curr : taquin courant
# n : taille du taquin
def move_function(curr,n):
    curr = curr.taquin
    for i in range(n):
        for j in range(n):
            if curr[i][j] == 0:
                x, y = i, j
                break
    q = []
    #Gauche
    if x-1 >= 0:
        b = deepcopy(curr)
        b[x][y]=b[x-1][y]
        b[x-1][y]=0
        succ = Taquin(n)
        succ.taquin = b
        succ.parent = curr
        q.append(succ)
    #Droite
    if x+1 < n:
        b = deepcopy(curr)
        b[x][y]=b[x+1][y]
        b[x+1][y]=0
        succ = Taquin(n)
        succ.taquin = b
        succ.parent = curr
        q.append(succ)
    #Bas
    if y-1 >= 0:
        b = deepcopy(curr)
        b[x][y]=b[x][y-1]
        b[x][y-1]=0
        succ = Taquin(n)
        succ.taquin = b
        succ.parent = curr
        q.append(succ)
    #Haut
    if y+1 < n:
        b = deepcopy(curr)
        b[x][y]=b[x][y+1]
        b[x][y+1]=0
        succ = Taquin(n)
        succ.taquin = b
        succ.parent = curr
        q.append(succ)

    return q

# retourne la meilleure valeur de f pour determiner le meilleur chemin
#openList : liste de noeuds
def best_fvalue(openList):
    f = openList[0].f
    index = 0
    for i, item in enumerate(openList):
        if i == 0: 
            continue
        if(item.f < f):
            f = item.f
            index  = i

    return openList[index], index

# Algorithme A*
# start: taquin de depart
# n : taille du taquin
# heuris: heuristique utilisee
def AStar(start,n,heuris):
    # creation des listes qui contiendront les noeuds
    openList = []
    closedList = []
    openList.append(start)

    while openList:
        current, index = best_fvalue(openList)
        if current.is_solution():
            return current
        openList.pop(index)
        closedList.append(current)

        # On recupere les voisins du noeud courant
        X = move_function(current,n)

        #Pour chaque voisins
        for move in X:
            # On regarde dans closedList si le noeud y est deja
            ok = False   
            for i, item in enumerate(closedList):
                if item == move:
                    ok = True
                    break
            # S'il n'est pas dans la liste
            if not ok:              
                newG = current.g + 1 
                present = False

                # on regarde si il est dans openList
                for j, item in enumerate(openList):
                    if item == move:
                        present = True
                        #si il y est, on vérifie sa valur, si elle est plus petite on met à jour la valur dans openList
                        if newG < openList[j].g:
                            openList[j].g = newG
                            openList[j].f = openList[j].g + openList[j].h
                            openList[j].parent = current

                 # sinon on calcule les couts avec heuristiques et on ajoute le noeud a openList         
                if not present:
                    move.g = newG
                    if heuris == 2:
                        move.h = move.manhattan(n)
                        move.f = move.g + move.h
                    if heuris == 1:
                        move.h = move.misplaced_Tiles(n)
                        move.f = move.h
                    move.parent = current
                    openList.append(move)
        

    return None

#########################################################################################
#
#     Algo UCS
#
#########################################################################################

def searchAlgorithm(start, n, heuris):
  
    nodes_expanded = 0
    max_queue_nodes = 0

    pq = []
    heappush(pq, start)

    goal = False
    tempList=Taquin(3)
    while not goal:

        if len(pq) == 0:
            print 'Frontier empty, failure to find goal state.'
            break

    # temp node = node courante dans queue
        heapify(pq)
        temp_node = heappop(pq)
        
    # vérifie si temp node = solution, si oui, fin
        if temp_node.is_solution():
            goal = True
            print 'Goal!'
            print_grid(temp_node)
            print 'To solve this problem the search algorithm expanded a total of ' , nodes_expanded , 'nodes.'
            print 'The maximum number of nodes in the queue at any one time was ' , max_queue_nodes , '.'
            print 'The depth of the goal node was ' , temp_node.g , '.'
            return temp_node

        else:
            tempList = move_function(temp_node, n)

    # met la node fille dans la file, et utilise heappush/heapify pour trier
    # MAJ heuristique et couts
            for x in range(len(tempList)):
                print tempList[x].taquin
                """if heuris == 1:
                    tempList[x].h = tempList[x].misplaced_Tiles(n)
                    tempList[x].f = tempList[x].h
                if heuris == 2:
                    tempList[x].h = tempList[x].manhattan(n)
                    tempList[x].f = tempList[x].h + tempList[x].g"""
                tempList[x].f = tempList[x].g

                heappush(pq, tempList[x])
    # MAJ max_queue_nodes
            if max_queue_nodes < len(pq):
                max_queue_nodes = len(pq)

#########################################################################################
#
#     Algo IDA*
#
#########################################################################################

# Algorithme ida*
# start: taquin de depart
# n : taille du taquin
# heuris: heuristique utilisee
def ida(start,n,heuris):
    bound = start.h
    #calcul de la limite en fonction des heuristiques
    if heuris==1:
        bound = start.misplaced_Tiles(n)
    if heuris==2:
        bound = start.manhattan(n)
    # initialisation de la liste de parents
    start.parent=[]
    start.parent.append(start.taquin)

    #tant que la solution n'est pas trouvée, on utilisa l'algorithme de recherche en profondeur
    while(1):
        t = search(start.parent,0,bound,n)
        if t==FOUND:
            return start.parent
        if t== sys.maxint:
            return NOT_FOUND
        bound =t 

# Algorithme de recherche en profondeur
# path: liste des noeuds parents
# g: valeur pour calculer la nouvelle limite
# bound: limite actuelle
# n : taille du taquin
def search(path,g,bound,n):
    node= Taquin(n)
    node.taquin=path[-1]
    #mise a jour de la limite
    if heuris==1:
        f=node.g+node.misplaced_Tiles(n)
    if heuris==2:
        f=node.g+node.manhattan(n)
    if node.f>bound:
        return node.f

    # verification de si le noeud est solution
    if node.is_solution():
        return FOUND

    min=sys.maxint
    #pour chaque sommet voisin
    for succ in move_function(node, n):
        present=False
        #verification si deja visite ou non
        for i in range(len(path)):
            if path[i]==succ:
                prensent=True
        #si non visite, mise a jour des parents
        if present==False:
            path.append(succ)
            t= search(path,f,bound)
            if t== FOUND:
                return FOUND
            if t< min:
                min=t
            path.pop()

    return min



#########################################################################################
#
#     MAIN
#
#########################################################################################

# Choix de la taille du taquin
n = input("Entrer la taille du taquin : ") 
while (n != 3) and (n != 4):
    print "La Taille du Taquin doit être 3 ou 4"
    n = input("Entrer la taille du taquin : ") 

# Initialisation du Taquin
t0 = Taquin(n)

# Generation d'une grille aléatoire
t0.generate_grid(n)

# Grilles de test
#t0.taquin = [[4,2,8],[5,3,7],[0,1,6]]
#t0.taquin = [[3,1,2],[0,4,5],[6,7,8]]
t0.taquin =[[4,7,0,5],[8,1,2,3],[12,9,15,6],[14,11,13,10]] 
#t0.taquin = [[1,5,3,7],[0,4,6,11],[12,8,2,4],[9,13,15,10]]
#t0.taquin = [[4,6,2,3],[1,15,7,10],[5,13,8,11],[0,12,9,14]]
#t0.taquin =[[4,1,2,3],[8,5,0,7],[9,13,6,11],[12,14,10,15]] 
#t0.taquin =[[4,1,2,3],[5,13,9,7],[12,10,6,11],[8,0,14,15]]
#t0.taquin = [[1,5,3,7],[4,6,2,11],[8,0,14,10],[12,9,13,15]]

# Affichage du Taquin initial
t0.print_grid()

# Vérification de la solubilité du Taquin
if(t0.isSolvable(n)) :      
    print  "Solvable" 
else: 
    print "Not Solvable"

################## A* + distance de Manhattan#######################################

# Initialisation calcul temps
start_time = time.time()

# Lancement de l'algorithme A*
result2 = AStar(t0,n,2)

# Compteur du nombre de déplacements
noofMoves = 0

# Calcul/Affichage du temps d'exécution
print("recherche avec A etoile et distance de manhattan terminee en %s secondes" % (time.time() - start_time))

# Affichage de la solution
if(not result2):
    print ("No solution")
else:
    print(result2.taquin)
    t=result2.parent
    while t:
        noofMoves += 1
        #print(t.taquin)  # Affichage des déplacements
        t=t.parent
print ("Nombre de mouvements: " + str(noofMoves))


################## A* + cases mal placées ####################################### 


# Initialisation calcul temps
start_time = time.time()

# Lancement de l'algorithme A*
result1 = AStar(t0,n,1)

# Compteur du nombre de déplacements
noofMoves = 0

# Calcul/Affichage du temps d'exécution
print("recherche A etoile et case mal placées terminee en %s secondes" % (time.time() - start_time))

# Affichage de la solution
if(not result1):
    print ("No solution")
else:
    print(result1.taquin)
    t=result1.parent
    while t:
        noofMoves += 1
        #print(t.taquin)    # Affichage des déplacements
        t=t.parent
print ("Nombre de mouvements: " + str(noofMoves))


################## UCS #######################################
"""
# Initialisation calcul temps
start_time = time.time()

# Lancement de l'algorithme UCS
result4 = searchAlgorithm(t0, n, 2)

# Compteur du nombre de déplacements
noofMoves = 0

# Calcul/Affichage du temps d'exécution
print("recherche avec UCS et distance de manhattan terminee en %s secondes" % (time.time() - start_time))

# Affichage de la solution
if(not result4):
    print ("No solution")
else:
    print(result4.taquin)

"""
################## IDA* + cases mal placées#######################################

"""
# Initialisation calcul temps
start_time = time.time()

# Lancement de l'algorithme IDA
result8 = ida(t0, n,1)

# Compteur du nombre de déplacements
noofMoves = 0

# Calcul/Affichage du temps d'exécution
print("recherche avec IDA et distance de manhattan terminee en %s secondes" % (time.time() - start_time))

# Affichage de la solution
if(not result8):
    print ("No solution")
else:
    print(result8.taquin)

"""

################## IDA* + distance Manhattan#######################################
"""
# Initialisation calcul temps
start_time = time.time()

# Lancement de l'algorithme IDA
result6 = ida(t0, n,2)

# Compteur du nombre de déplacements
noofMoves = 0

# Calcul/Affichage du temps d'exécution
print("recherche avec IDA et distance de manhattan terminee en %s secondes" % (time.time() - start_time))

# Affichage de la solution
if(not result6):
    print ("No solution")
else:
    print(result6.taquin)

"""
