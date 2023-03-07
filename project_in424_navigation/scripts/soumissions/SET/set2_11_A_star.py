# -*- coding: utf-8 -*-
"""
Created on Mon Mar 28 08:34:20 2022

@author: luca5
"""


class Spot():

    def __init__(self, i, j):
        self.neighbors = []
        self.previous = None
        self.i = i
        self.j = j
        self.g = 0
        self.h = 0
        self.f = 0
        self.wall = False


def heuristic(a, b, c, d):
    return abs(a-b)+abs(c-d)


def astar(tab,tab1,start,end,rows,cols):
    
    retour=0
    path=[]
    closed_set=[]
    open_set=[]
    tab=tab
    tab_m=tab1
    
    
    for i in range(rows):
        for j in range(cols):
            tab_m[i][j]=Spot(i,j)
            if (tab[i][j]==2):
                tab_m[i][j].wall=True
    """          
    for i in range(rows):
        for j in range(cols):
            print("("+str(tab_m[i][j].i)+","+str(tab_m[i][j].j)+")\t")
        print("\n")
    """  
    for i in range(rows):
        
        for j in range(cols):
            
            if(i < rows-1):
                
                tab_m[i][j].neighbors.append(tab_m[i+1][j])

            if(i > 0):
                
                tab_m[i][j].neighbors.append(tab_m[i-1][j])

            if(j < cols-1):
                
                tab_m[i][j].neighbors.append(tab_m[i][j+1])
                
            if(j > 0):
                
                tab_m[i][j].neighbors.append(tab_m[i][j-1])
            
 
    start_node = tab_m[start[0]][start[1]]
    end_node = tab_m[end[0]][end[1]]
    tab_m[start_node.i][start_node.j].g = 0
    tab_m[start_node.i][start_node.j].h = heuristic(start_node.i,end_node.i,start_node.j,end_node.j)
    tab_m[start_node.i][start_node.j].f = tab_m[start_node.i][start_node.j].g+tab_m[start_node.i][start_node.j].f
    tab_m[start_node.i][start_node.j].wall=False
    tab_m[end_node.i][end_node.j].wall=False
    open_set.append(tab_m[start_node.i][start_node.j])
    
    while(retour==0):
        
        if len(open_set)>0:
            
            lowest_index=0
            
            for i in range(len(open_set)):
                if (open_set[i].f<open_set[lowest_index].f):
                    lowest_index=i
                    
            current=tab_m[open_set[lowest_index].i][open_set[lowest_index].j]
            
            if(current.i==end_node.i and current.j==end_node.j):
                temp=tab_m[current.i][current.j]
                path.append(temp)
                
                while(temp.previous):
                    path.append(temp.previous)
                    temp=temp.previous
                
                for i in range(rows):
                    for j in range(cols):
                        if(tab_m[i][j].wall==True):
                            tab[i][j]=2
                            tab_m[i][j]=2
                        else:
                            tab[i][j]=0
                            tab_m[i][j]=0
                        for k in range(len(path)):
                            if(path[k].i==i and path[k].j==j):
                                tab_m[i][j]=int(1)
                      
                            
                return tab_m,tab  
                retour=1 
            open_set.remove(current)
            closed_set.append(current)
            
            neighbors=tab_m[current.i][current.j].neighbors
            
            for i in range(len(neighbors)):
                neighbor=tab_m[neighbors[i].i][neighbors[i].j]
                
                if (not (neighbor in closed_set) and not tab_m[neighbor.i][neighbor.j].wall):
                    
                    tempG=tab_m[current.i][current.j].g+1
                    
                    if(neighbor in open_set):
                        
                        if (tempG<neighbor.g):
                            
                            tab_m[neighbor.i][neighbor.j].g=tempG;
                    else:
                        tab_m[neighbor.i][neighbor.j].g=tempG
                        open_set.append(neighbor)
                        
                    tab_m[neighbor.i][neighbor.j].h=heuristic(tab_m[neighbor.i][neighbor.j].i,tab_m[end_node.i][end_node.j].i,tab_m[neighbor.i][neighbor.j].j,tab_m[end_node.i][end_node.j].j)
                    tab_m[neighbor.i][neighbor.j].f=tab_m[neighbor.i][neighbor.j].h + tab_m[neighbor.i][neighbor.j].g
                    tab_m[neighbor.i][neighbor.j].previous=tab_m[current.i][current.j]
                    
        else:
            print("Pas de solution")
            retour=1




def matric(rows, cols):
    maze = []

    inte = []
    for j in range(rows):
        maze.append(None)
    for i in range(rows):
        inte = []
        for j in range(cols):
            inte.append(0)
        maze[i] = inte
    return maze


if __name__ == '__main__':
    rows = 10
    cols = 10

    maze = matric(rows, cols)

    maze1 = matric(rows, cols)

    start = [4, 4]
    end = [9, 9]
    maze1 = astar(maze, maze1, start, end, rows, cols)
    
