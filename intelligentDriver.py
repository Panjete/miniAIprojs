'''
Licensing Information: Please do not distribute or publish solutions to this
project. You are free to use and extend Driverless Car for educational
purposes. The Driverless Car project was developed at Stanford, primarily by
Chris Piech (piech@cs.stanford.edu). It was inspired by the Pacman projects.
'''
import util
import itertools
from turtle import Vec2D
from engine.const import Const
from engine.vector import Vec2d
from engine.model.car.car import Car
from engine.model.layout import Layout
from engine.model.car.junior import Junior
from configparser import InterpolationMissingOptionError
import random
from math import *

# Class: Graph
# -------------
# Utility class
class Graph(object):
    def __init__(self, nodes, edges):
        self.nodes = nodes
        self.edges = edges


def flatten(l):
    return [item for sublist in l for item in sublist]

def isSafe(row,col,grids,k=1,min_prob=0.02):
    # print(grids)
    for row_new in range(row-k,row+k+1):
        for col_new in range(col-k,col+k+1):
            if row_new < 0 or row_new >= len(grids[0]) or col_new < 0 or col_new >= len(grids[0][0]):
                continue
            cur_prob =0.0
            for grid in grids:
                cur_prob = max(grid[row_new][col_new]*pow(0.5,k),cur_prob)
            
            if cur_prob > min_prob:
                # print("not safe \n\n")
                return False
    # print(row,col,cur_prob)
    return True

def returnProb(row,col,grids,k=1):
    # print(grids)
    max_prob=0.0
    for row_new in range(row-k,row+k+1):
        for col_new in range(col-k,col+k+1):
            if row_new < 0 or row_new >= len(grids[0]) or col_new < 0 or col_new >= len(grids[0][0]):
                continue
            for grid in grids:
                max_prob = max(grid[row_new][col_new]*pow(0.5,k),max_prob)
    # print(row,col,cur_prob)
    return max_prob

def returnProb_avg(row,col,grids,k=1):
    # print(grids)
    avg_prob=0.0
    for row_new in range(row-k,row+k+1):
        for col_new in range(col-k,col+k+1):
            if row_new < 0 or row_new >= len(grids[0]) or col_new < 0 or col_new >= len(grids[0][0]) or abs(row_new-row) + abs(col_new-col) > k:
                continue
            cur_prob =0.0
            for grid in grids:
                cur_prob = max(grid[row_new][col_new],cur_prob)
            avg_prob += cur_prob
    
    avg_prob /= pow(k+1,2)
    # print(row,col,cur_prob)
    return avg_prob


def get_dist(particle,goal,graph):
    #implement bfs for finding the shortest path
    cont =[particle]
    dist =0
    visited = set()
    while cont:
        new_cont = []
        for node in cont:
            if node == goal:
                return dist
            if node not in visited:
                visited.add(node)
                new_cont.extend(graph.edges[node])
        cont = new_cont
        dist += 1
    return dist

def next_state_of_particles(particle_list_old,transProb,numRows,numCols,isParked: bool, no_of_particles: int):
        particle_list = particle_list_old.copy()
        prior_belief = util.Belief(numRows, numCols, 0.0)
        if not isParked :
            #create count of each cell
            count_particles = {}
            for particle in particle_list:
                if particle in count_particles:
                    count_particles[particle] += 1
                else:
                    count_particles[particle] = 1



            for particle in count_particles.keys():
                for row in range(numRows):
                    for col in range(numCols):
                        trans_prob = 0.0
                        if (((particle[0], particle[1]),(row, col)) in transProb):
                            trans_prob = transProb[((particle[0], particle[1]),(row, col))] 
                            # print("there it is")
                        prior_belief.addProb(row, col, trans_prob*count_particles[particle])
                        #add noise

            prior_belief.normalize()
        
        else:
            for particle in particle_list:
                prior_belief.addProb(particle[0], particle[1], 1.0)
            prior_belief.normalize()

        while len(particle_list) < no_of_particles:
            particle_list.append((random.randint(0, numRows - 1), random.randint(0, numCols - 1)))

        new_belief = prior_belief
        new_belief.normalize()

        agg_row_col = random.choices([i for i in range(numRows * numCols)], weights =flatten(new_belief.grid), k = no_of_particles)


        for i in range(no_of_particles):
            particle_list[i] = (agg_row_col[i] // numCols, agg_row_col[i] % numCols)
        
        return particle_list

def generate_belief_from_particle_list(particle_list, numRows, numCols):
    belief = util.Belief(numRows, numCols, 0.0)
    for particle in particle_list:
        belief.addProb(particle[0], particle[1], 1.0)
    belief.normalize()
    return belief

def get_particle_list(prob_grid,no_of_particles):
    particle_list = []
    num_rows = len(prob_grid)
    num_cols = len(prob_grid[0])
    for i in range(num_rows):
        for j in range(num_cols):
            particle_list.extend([(i,j)]*ceil(no_of_particles*prob_grid[i][j]))
    return particle_list


# Class: IntelligentDriver
# ---------------------
# An intelligent driver that avoids collisions while visiting the given goal locations (or checkpoints) sequentially. 
class IntelligentDriver(Junior):

    # Funciton: Init
    def __init__(self, layout: Layout):
        self.burnInIterations = 50
        self.layout = layout 
        self.distGraph = self.createDistGraph()
        self.worldGraph = self.createWorldGraph()
        self.checkPoints = self.layout.getCheckPoints() # a list of single tile locations corresponding to each checkpoint
        self.transProb = util.loadTransProb()
        self.blockTiles = []
        self.maxit =1000
        
    # ONE POSSIBLE WAY OF REPRESENTING THE GRID WORLD. FEEL FREE TO CREATE YOUR OWN REPRESENTATION.
    # Function: Create World Graph
    # ---------------------
    # Using self.layout of IntelligentDriver, create a graph representing the given layout.



    def get_unblocked_neib(self,row,col):
        blocks = self.layout.getBlockData()
        blockTiles = []
        for block in blocks:
            row1, col1, row2, col2 = block[1], block[0], block[3], block[2] 
            blockTiles.extend([(i,j) for i in range(row1,row2) for j in range(col1,col2)])
        unblocked_neib = []

        numRows = self.layout.getBeliefRows()
        numCols = self.layout.getBeliefCols()
        for i in range(row-1,row+2):
            for j in range(col-1,col+2):
                if (i,j) not in blockTiles and (i,j) != (row,col) and i >= 0 and j >= 0 and i < numRows and j < numCols and abs(i-row) + abs(j-col) == 1:
                    unblocked_neib.append((i,j))
        return unblocked_neib


    def createWorldGraph(self):
        nodes = []
        #init edges as adjacency matrix(dictionary)
        edges = {}

        # create self.worldGraph using self.layout
        numRows, numCols = self.layout.getBeliefRows(), self.layout.getBeliefCols()
        # print(numRows,numCols)
        # NODES #
        ## each tile represents a node
        nodes = [(x, y) for x, y in itertools.product(range(numRows), range(numCols))]
        
        # EDGES #
        ## We create an edge between adjacent nodes (nodes at a distance of 1 tile)
        ## avoid the tiles representing walls or blocks#
        ## YOU MAY WANT DIFFERENT NODE CONNECTIONS FOR YOUR OWN IMPLEMENTATION,
        ## FEEL FREE TO MODIFY THE EDGES ACCORDINGLY.

        ## Get the tiles corresponding to the blocks (or obstacles):
        blocks = self.layout.getBlockData()
        # print(blocks)
        blockTiles = []
        for block in blocks:
            row1, col1, row2, col2 = block[1], block[0], block[3], block[2] 


            # some padding to ensure the AutoCar doesn't crash into the blocks due to its size. (optional)
            row1, col1, row2, col2 = row1-1, col1-1, row2+1, col2+1
            blockWidth = col2-col1 
            blockHeight = row2-row1

            for i in range(row1,row2):
                for j in range(col1,col2):
                    blockTile = (i, j)
                    blockTiles.append(blockTile)
            # add end tiles into blockTiles

            self.blockTiles = blockTiles.copy()
            endTiles =[]
            for i in range(numRows):
                endTiles.append((i,0))
                endTiles.append((i,numCols-1))
            for i in range(numCols):
                endTiles.append((0,i))
                endTiles.append((numRows-1,i))
            

        ## Remove blockTiles from 'nodes'
        nodes = [x for x in nodes if x not in blockTiles]
        edges = {}
        #initialize edges
        for node in nodes:
            edges[node] = []
        for node in nodes:
            x, y = node[0], node[1]
            adjNodes = [(x, y-1), (x, y+1), (x-1, y), (x+1, y)]
            # (x-1, y-1), (x+1, y+1), (x-1, y+1), (x+1, y-1)
            
            # only keep allowed (within boundary) adjacent nodes
            adjacentNodes = []
            for tile in adjNodes:
                if tile[0]>=0 and tile[1]>=0 and tile[0]<numRows and tile[1]<numCols:
                    if tile not in blockTiles:
                        adjacentNodes.append(tile)

            edges[node] = adjacentNodes.copy()
            # print(node,adjacentNodes)
        return Graph(nodes, edges)

    def createDistGraph(self):
        nodes = []
        #init edges as adjacency matrix(dictionary)
        edges = {}

        # create self.worldGraph using self.layout
        numRows, numCols = self.layout.getBeliefRows(), self.layout.getBeliefCols()
        # print(numRows,numCols)
        # NODES #
        ## each tile represents a node
        nodes = [(x, y) for x, y in itertools.product(range(numRows), range(numCols))]
        
        # EDGES #
        ## We create an edge between adjacent nodes (nodes at a distance of 1 tile)
        ## avoid the tiles representing walls or blocks#
        ## YOU MAY WANT DIFFERENT NODE CONNECTIONS FOR YOUR OWN IMPLEMENTATION,
        ## FEEL FREE TO MODIFY THE EDGES ACCORDINGLY.

        ## Get the tiles corresponding to the blocks (or obstacles):
        blocks = self.layout.getBlockData()
        # print(blocks)
        blockTiles = []
        for block in blocks:
            row1, col1, row2, col2 = block[1], block[0], block[3], block[2] 
            # some padding to ensure the AutoCar doesn't crash into the blocks due to its size. (optional)
            row1, col1, row2, col2 = row1-1, col1-1, row2+1, col2+1
            blockWidth = col2-col1 
            blockHeight = row2-row1

            for i in range(row1,row2):
                for j in range(col1,col2):
                    blockTile = (i, j)
                    blockTiles.append(blockTile)
            # add end tiles into blockTiles


            

        ## Remove blockTiles from 'nodes'
        nodes = [x for x in nodes if x not in blockTiles]
        edges = {}
        #initialize edges
        for node in nodes:
            edges[node] = []
        for node in nodes:
            x, y = node[0], node[1]
            adjNodes = [(x, y-1), (x, y+1), (x-1, y), (x+1, y)]
            # (x-1, y-1), (x+1, y+1), (x-1, y+1), (x+1, y-1)
            
            # only keep allowed (within boundary) adjacent nodes
            adjacentNodes = []
            for tile in adjNodes:
                if tile[0]>=0 and tile[1]>=0 and tile[0]<numRows and tile[1]<numCols:
                    if tile not in blockTiles:
                        adjacentNodes.append(tile)

            edges[node] = adjacentNodes.copy()
            # print(node,adjacentNodes)
        return Graph(nodes, edges)

    #######################################################################################
    # Function: Get Next Goal Position
    # ---------------------
    # Given the current belief about where other cars are and a graph of how
    # one can driver around the world, chose the next position.
    #######################################################################################
    def getNextGoalPos(self, beliefOfOtherCars: list, parkedCars:list , chkPtsSoFar: int):
        '''
        Input:
        - beliefOfOtherCars: list of beliefs corresponding to all cars
        - parkedCars: list of booleans representing which cars are parked
        - chkPtsSoFar: the number of checkpoints that have been visited so far 
                       Note that chkPtsSoFar will only be updated when the checkpoints are updated in sequential order!
        
        Output:
        - goalPos: The position of the next tile on the path to the next goal location.
        - moveForward: Unset this to make the AutoCar stop and wait.

        Notes:
        - You can explore some files "layout.py", "model.py", "controller.py", etc.
         to find some methods that might help in your implementation. 
        '''
        #find next checkpoint
        if self.burnInIterations > 0:
            return (0,0),False
        # print(chkPtsSoFar)
        goal = self.checkPoints[chkPtsSoFar]
        goal = (goal[0],goal[1])
        # print("goal is",goal)
        no_of_particles = 1000
        numRows = self.layout.getBeliefRows()
        numCols = self.layout.getBeliefCols()
        # select randomly from the list of possible adjacent edges
        # print(self.getPos())  
        curCol = util.xToCol(self.getPos()[0])
        curRow = util.yToRow(self.getPos()[1])
        start = (curRow, curCol)
        # print("start tile",start)
        # print("backward tile",self.getBackTile())
        search_list =[]
        if( start in self.worldGraph.edges):
            search_list = self.worldGraph.edges[start].copy()
        #add start to search list
            search_list.append(start)
        else:
            # print("fas gaya :::::00000")
            # search_list = self.get_unblocked_neib(start[0],start[1])
            adj_list = [(start[0],start[1]-1),(start[0],start[1]+1),(start[0]-1,start[1]),(start[0]+1,start[1])]
            for adj in adj_list:
                if adj in self.worldGraph.nodes:
                    search_list.append(adj)
            if len(search_list) == 0:
                # print("lmfao")
                next_goal = self.get_unblocked_neib(start[0],start[1])[0]
                return (util.rowToY(next_goal[0]),util.colToX(next_goal[1])),True
            # ,(start[0]-1, start[1]-1), (start[0]+1, start[1]+1), (start[0]-1, start[1]+1), (start[0]+1, start[1]-1)

        # print(search_list,"::::::::::::")
        self.prev = start
        # select the next goal position from search list having minimum distance from the goal position and maximum number of elements in the search list
        min_dist = inf
        max_num = 0
        next_goal = start
        particle_lists= [get_particle_list(belief.grid,no_of_particles) for belief in beliefOfOtherCars]
        next_particle_lists = [next_state_of_particles(particle_list, self.transProb,numRows,numCols,parkedCars[i],no_of_particles) for i,particle_list in enumerate(particle_lists)]
        prob_grid = [generate_belief_from_particle_list(particle_list,numRows,numCols).grid for particle_list in next_particle_lists]


        cur_node_prob = returnProb(start[0],start[1],prob_grid)
        min_prob = cur_node_prob
        for cur_node in search_list:
            if cur_node_prob>0.01:
                adj_node_prob = returnProb(cur_node[0],cur_node[1],prob_grid)
                if(adj_node_prob<min_prob):
                    min_prob = adj_node_prob
                    next_goal = cur_node
            else:
                if(isSafe(cur_node[0],cur_node[1],prob_grid)):
                    dist_obt = get_dist(cur_node,goal,self.distGraph)
                    # print(cur_node,dist_obt)
                    adj_node_prob = returnProb(cur_node[0],cur_node[1],prob_grid)
                    if dist_obt < min_dist:
                        min_dist = dist_obt
                        next_goal = cur_node
                        min_prob = adj_node_prob
                    elif dist_obt == min_dist:
                        if adj_node_prob < min_prob:
                            min_prob = adj_node_prob
                            next_goal = cur_node

        moveForward = True

        if(start == next_goal):
            moveForward = False
        
        # if(next_goal[1]== 0):
        #     next_goal = (next_goal[0],1)
        # elif(next_goal[1]== numCols-1):
        #     next_goal = (next_goal[0],numCols-2)
        # elif(next_goal[0]== 0):
        #     next_goal = (1,next_goal[1])
        # elif(next_goal[0]== numRows-1):
        #     next_goal = (numRows-2,next_goal[1])



        # looka2 = (2*next_goal[0] - util.yToRow(self.getPos()[1]), 2*next_goal[1]- util.xToCol(self.getPos()[0]))
        # if looka2 not in self.worldGraph.nodes:
        #     goalPos = (util.rowToY(next_goal[1])*0.5 +(self.getPos()[0])*0.5,util.colToX(next_goal[0])*0.5+(self.getPos()[1])*0.5)
        #     num = random.random()
        #     print(num)
        #     if num < 0.8:  
        #         return goalPos, False

        # if(moveForward==False):
        #     self.maxit-=1
        # else:
        #     self.maxit = 1000

        # if(self.maxit == 0):
        #     print("move forward")
        #     self.maxit = 1000
        #     moveForward = True
        #     next_goal = goal
        #     goalPos = (util.rowToY(next_goal[0]),util.colToX(next_goal[1]))
        #     return goalPos, moveForward

        if(next_goal[1]== 0):
            goalPos = (util.rowToY(next_goal[1])*0.5 +(self.getPos()[0])*0.5,util.colToX(1)-util.colToX(0))
        elif(next_goal[1]== numCols-1):
            goalPos = (util.rowToY(next_goal[1])*0.5 +(self.getPos()[0])*0.5, util.colToX(numCols-1-1)-util.colToX(0))
        elif(next_goal[0]== 0): #y axis
            goalPos = (util.rowToY(1)-util.rowToY(0), util.colToX(next_goal[0])*0.5+(self.getPos()[1])*0.5)
        elif(next_goal[0]== numRows-1):
            goalPos = (util.rowToY(numRows-1-1)-util.rowToY(0),util.colToX(next_goal[0])*0.5+(self.getPos()[1])*0.5)
        else:
            goalPos = (util.rowToY(next_goal[1])*0.5 +(self.getPos()[0])*0.5,util.colToX(next_goal[0])*0.5+(self.getPos()[1])*0.5)
        # BEGIN_YOUR_CODE 

        # END_YOUR_CODE
        return goalPos, moveForward

    # DO NOT MODIFY THIS METHOD !
    # Function: Get Autonomous Actions
    # --------------------------------
    def getAutonomousActions(self, beliefOfOtherCars: list, parkedCars: list, chkPtsSoFar: int):
        # Don't start until after your burn in iterations have expired
        if self.burnInIterations > 0:
            self.burnInIterations -= 1
            return[]
       
        goalPos, df = self.getNextGoalPos(beliefOfOtherCars, parkedCars, chkPtsSoFar)
        vectorToGoal = goalPos - self.pos
        wheelAngle = -vectorToGoal.get_angle_between(self.dir)
        driveForward = df
        actions = {
            Car.TURN_WHEEL: wheelAngle
        }
        if driveForward:
            actions[Car.DRIVE_FORWARD] = 1.0
        return actions

                    
    
    