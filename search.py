# Basic searching algorithms
import heapq as heap

# Class for each node in the grid
class Node:
    def __init__(self, row, col, is_obs, h):
        self.row = row        # coordinate
        self.col = col        # coordinate
        self.is_obs = is_obs  # obstacle?
        self.g = None         # cost to come (previous g + moving cost)
        self.h = h            # heuristic
        self.cost = None      # total cost (depend on the algorithm)
        self.parent = None    # previous node

def dijkstra(grid, start, goal):
    '''Return a path found by Dijkstra alogirhm 
       and the number of steps it takes to find it.

    arguments:
    grid - A nested list with datatype int. 0 represents free space while 1 is obstacle.
           e.g. a 3x3 2D map: [[0, 0, 0], [0, 1, 0], [0, 0, 0]]
    start - The start node in the map. e.g. [0, 0]
    goal -  The goal node in the map. e.g. [2, 2]

    return:
    path -  A nested list that represents coordinates of each step (including start and goal node), 
            with data type int. e.g. [[0, 0], [0, 1], [0, 2], [1, 2], [2, 2]]
    steps - Number of steps it takes to find the final solution, 
            i.e. the number of nodes visited before finding a path (including start and goal node)

    >>> from main import load_map
    >>> grid, start, goal = load_map('test_map.csv')
    >>> dij_path, dij_steps = dijkstra(grid, start, goal)
    It takes 10 steps to find a path using Dijkstra
    >>> dij_path
    [[0, 0], [1, 0], [2, 0], [3, 0], [3, 1]]
    '''
    ### YOUR CODE HERE ###
    path = []
    steps = 0
    found = False
    visited = [[False for i in range(len(grid))] for j in range(len(grid[0]))]
    parent = [[0 for i in range(len(grid))] for j in range(len(grid[0]))]
    dist = [[float('inf') for i in range(len(grid))] for j in range(len(grid[0]))]


    dist[start[0]][start[1]] = 0
 
    h = [(0,start[0],start[1])]
   
    while len(h)>0:
        d,x,y=heap.heappop(h)
       
        if not visited[x][y]:
            visited[x][y]=True
            steps=steps+1 
            if [x,y]==goal:
                found=True
                break
            if d > dist[x][y]:
                continue
            

            for dx,dy in [(0,1),(1,0),(0,-1),(-1,0)]:
                nx,ny = x+dx,y+dy
                if 0<=nx and nx<len(grid) and 0<=ny and ny<len(grid[0]) and grid[nx][ny]!=1 and not visited[nx][ny]:
                    parent[nx][ny]=[x,y]
                    new=d+1  
                    
                    if new<dist[nx][ny]:
                        dist[nx][ny]=new
                        heap.heappush(h,(new,nx,ny))
                    
    if found:
        node = goal
        while node!=start:
            path.append(node)
            node=parent[node[0]][node[1]]
        path.append(node)
        path.reverse()
   
        print(f"It takes {steps} steps to find a path using Dijkstra")
    else:
        print("No path found")
    return path, steps


def astar(grid, start, goal):
    '''Return a path found by A* alogirhm 
       and the number of steps it takes to find it.

    arguments:
    grid - A nested list with datatype int. 0 represents free space while 1 is obstacle.
           e.g. a 3x3 2D map: [[0, 0, 0], [0, 1, 0], [0, 0, 0]]
    start - The start node in the map. e.g. [0, 0]
    goal -  The goal node in the map. e.g. [2, 2]

    return:
    path -  A nested list that represents coordinates of each step (including start and goal node), 
            with data type int. e.g. [[0, 0], [0, 1], [0, 2], [1, 2], [2, 2]]
    steps - Number of steps it takes to find the final solution, 
            i.e. the number of nodes visited before finding a path (including start and goal node)

    >>> from main import load_map
    >>> grid, start, goal = load_map('test_map.csv')
    >>> astar_path, astar_steps = astar(grid, start, goal)
    It takes 7 steps to find a path using A*
    >>> astar_path
    [[0, 0], [1, 0], [2, 0], [3, 0], [3, 1]]
    '''
    ### YOUR CODE HERE ###
    path = []
    steps = 0
    found = False
    visited = [[False for i in range(len(grid))] for j in range(len(grid[0]))]
    parent = [[0 for i in range(len(grid))] for j in range(len(grid[0]))]
    dist = [[float('inf') for i in range(len(grid))] for j in range(len(grid[0]))]

    dist[start[0]][start[1]] = 0
    # To keep count of the stored value in the heap so as to priortise the first value when more than one node have the similiar fscores(distance+heuristics)
    ct = 0
    h = [(0,0,start[0],start[1])]
     
    while len(h)>0:
              
        f,_,x,y=heap.heappop(h)
        
        if not visited[x][y]:
            visited[x][y]=True
            steps=steps+1
        
            if [x,y]==goal:
                found=True
                break
                      
            for dx,dy in [(0,1),(1,0),(0,-1),(-1,0)]:
                nx,ny = x+dx,y+dy
                heuristic=abs((goal[0]-nx))+abs((goal[1]-ny))             
                
                if 0<=nx and nx<len(grid) and 0<=ny and ny<len(grid[0]) and grid[nx][ny]!=1 and not visited[nx][ny]:# and parent[nx][ny]==0:
                                     
                    new=dist[x][y]+1  
                    
                    if new<dist[nx][ny]:
                        parent[nx][ny]=[x,y]
                        dist[nx][ny]=new
                        ct+=1
                        f=new+heuristic
                        heap.heappush(h,(f,ct,nx,ny))
    
    if found:
        node = goal
        while node!=start:
            path.append(node)
            node=parent[node[0]][node[1]]
        path.append(node)
        path.reverse()
        print(f"It takes {steps} steps to find a path using A*")
    else:
        print("No path found")
    return path, steps


# Doctest
if __name__ == "__main__":
    # load doc test
    from doctest import testmod, run_docstring_examples
    # Test all the functions
    testmod()