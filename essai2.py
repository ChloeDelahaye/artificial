# -*- coding:Utf8 -*-
import time
from copy import deepcopy

COEFF = (4, 1)  #rho1 à rho6

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
    def generate_grid(self,n):
        node=[i for i in range(n*n)]
        print node
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
    def getInvCount(self,arr,n): 
        inv_count = 0
        for i in range((n*n)-1): 
            for j in range (i+1,(n*n)):
                 
                if (arr[i] and arr[j] and arr[i] > arr[j]):
                    inv_count=inv_count+1

            
        return inv_count
    
    # Dertermine la position du zero dans la grille initiale par rapport à dans la grille finale
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
    def isSolvable(self,n):
        arr=[]
        for i in range(n):
            for j in range(n):
                arr.append(self.taquin[i][j])
        print arr

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
    def manhattan(self,n):
        h = 0
        for i in range(n):
            for j in range(n):
                x, y = divmod(self.taquin[i][j], n)
                h += abs(x-i) + abs(y-j)
        return h


    def __eq__(self, other):
        return self.taquin == other.taquin

    # Calcul du nombre de cases mal placées
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
def AStar(start,n,heuris):
    openList = []
    closedList = []
    openList.append(start)

    while openList:
        current, index = best_fvalue(openList)
        if current.is_solution():
            return current
        openList.pop(index)
        closedList.append(current)

        X = move_function(current,n)
        for move in X:
            ok = False   #regarde dans closedList
            for i, item in enumerate(closedList):
                if item == move:
                    ok = True
                    break
            if not ok:              #si pas dans closed list
                newG = current.g + 1 
                present = False

                
                for j, item in enumerate(openList):
                    if item == move:
                        present = True
                        if newG < openList[j].g:
                            openList[j].g = newG
                            openList[j].f = openList[j].g + openList[j].h
                            openList[j].parent = current
                            
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
        #print current.taquin

    return None

#########################################################################################
#
#     Algo UCS
#
#########################################################################################











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
#t0.taquin = [[5,0,1],[4,3,6],[7,2,8]]
#t0.taquin = [[3,1,2],[0,4,5],[6,7,8]]
#t0.taquin =[[15,0,1,2],[4,5,7,3],[8,9,6,11],[12,13,10,14]] 
#t0.taquin = [[3,9,1,15],[14,11,4,6],[13,0,10,12],[2,7,8,5]]
#t0.taquin = [[6,13,7,10],[8,9,11,0],[15,2,12,5],[14,3,1,4]]
#t0.taquin = [[4,1,2,3],[5,0,6,7],[8,9,10,11],[12,13,14,15]]

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
print("recherche terminee en %s secondes" % (time.time() - start_time))

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
print ("Length: " + str(noofMoves))


################## A* + cases mal placées ####################################### 


# Initialisation calcul temps
start_time = time.time()

# Lancement de l'algorithme A*
result1 = AStar(t0,n,1)

# Compteur du nombre de déplacements
noofMoves = 0

# Calcul/Affichage du temps d'exécution
print("recherche terminee en %s secondes" % (time.time() - start_time))

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
print ("Length: " + str(noofMoves))




