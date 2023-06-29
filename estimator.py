import util 
from util import Belief, pdf 
from engine.const import Const
from math import *
import random

def dist(x1, y1, x2, y2):
    return sqrt((x1 - x2) ** 2 + (y1 - y2) ** 2)

def flatten(l):
    return [item for sublist in l for item in sublist]
# Class: Estimator
#----------------------
# Maintain and update a belief distribution over the probability of a car being in a tile.
class Estimator(object):
    def __init__(self, numRows: int, numCols: int):
        self.belief = util.Belief(numRows, numCols) 
        self.transProb = util.loadTransProb() 
        self.particle_no = 5000
        self.particle_list = [(row,col) for row in range(numRows) for col in range(numCols) ]
        self.it =50
            
    ##################################################################################
    # [ Estimation Problem ]
    # Function: estimate (update the belief about a StdCar based on its observedDist)
    # ----------------------
    # Takes |self.belief| -- an object of class Belief, defined in util.py --
    # and updates it *inplace* based onthe distance observation and your current position.
    #
    # - posX: x location of AutoCar 
    # - posY: y location of AutoCar 
    # - observedDist: current observed distance of the StdCar 
    # - isParked: indicates whether the StdCar is parked or moving. 
    #             If True then the StdCar remains parked at its initial position forever.
    # 
    # Notes:
    # - Carefully understand and make use of the utilities provided in util.py !
    # - Remember that although we have a grid environment but \
    #   the given AutoCar position (posX, posY) is absolute (pixel location in simulator window).
    #   You might need to map these positions to the nearest grid cell. See util.py for relevant methods.
    # - Use util.pdf to get the probability density corresponding to the observedDist.
    # - Note that the probability density need not lie in [0, 1] but that's fine, 
    #   you can use it as probability for this part without harm :)
    # - Do normalize self.belief after updating !!

    ###################################################################################

    def exact_inference(self, posX: float, posY: float, observedDist: float, isParked: bool) -> None:
        auto_car_row = util.yToRow(posY)
        auto_car_col = util.xToCol(posX)

        #update the belief using transition probability
        # all zero belief

        if not isParked:
            new_belief = util.Belief(self.belief.numRows, self.belief.numCols, 0.0)
            for new_row in range(self.belief.numRows):
                for new_col in range(self.belief.numCols):
                    #add transition probability
                    for old_row in range(self.belief.numRows):
                        for old_col in range(self.belief.numCols):
                            trans_prob = 0.0
                            if (((old_row, old_col),(new_row, new_col)) in self.transProb):
                                trans_prob = self.transProb[((old_row, old_col),(new_row, new_col))]
                                print("there it is")
                            new_belief.addProb(new_row, new_col, self.belief.getProb(old_row, old_col) * trans_prob)
            self.belief = new_belief
            self.belief.normalize()

        for row in range(self.belief.numRows):
            for col in range(self.belief.numCols):
                std_car_x = util.colToX(col)
                std_car_y = util.rowToY(row)
                std_car_dist = dist(std_car_x, std_car_y, posX, posY)
                self.belief.grid[row][col] *= pdf(std_car_dist, Const.SONAR_STD, observedDist)
        self.belief.normalize()
        return



    def particle_filtering(self, posX: float, posY: float, observedDist: float, isParked: bool, no_of_particles: int) -> None:
        # BEGIN_YOUR_CODE
        # implement Particle Filtering
        particle_list = self.particle_list


        prior_belief = util.Belief(self.belief.numRows, self.belief.numCols, 0.0)
        count_particles = {}
        for particle in particle_list:
            if particle in count_particles:
                count_particles[particle] += 1
            else:
                count_particles[particle] = 1
        if not isParked :
            #create count of each cell
            for particle in count_particles.keys():
                for row in range(self.belief.numRows):
                    for col in range(self.belief.numCols):
                        trans_prob = 0.0
                        if (((particle[0], particle[1]),(row, col)) in self.transProb):
                            trans_prob = self.transProb[((particle[0], particle[1]),(row, col))] 
                            # print("there it is")
                        prior_belief.addProb(row, col, trans_prob*count_particles[particle])
                        #add noise

            prior_belief.normalize()
        
        else:
            k =0
            for particle in count_particles.keys():
                for row in range(particle[0]-k,particle[0]+k+1):
                    for col in range(particle[1]-k,particle[1]+k+1):
                        if row >= 0 and row < self.belief.numRows and col >= 0 and col < self.belief.numCols:
                            curdis = abs(row-particle[0]) + abs(col-particle[1])
                            prior_belief.addProb(row, col, count_particles[particle]*(pow(0.005,curdis)))
            prior_belief.normalize()

        while len(particle_list) < no_of_particles:
            particle_list.append((random.randint(0, self.belief.numRows - 1), random.randint(0, self.belief.numCols - 1)))

        # update using the evidence

        # if(self.it>0):
        for row in range(self.belief.numRows):
            for col in range(self.belief.numCols):

                std_car_x = util.colToX(col)
                std_car_y = util.rowToY(row)
                std_car_dist = dist(std_car_x, std_car_y, posX, posY)
                prior_belief.grid[row][col] *= pdf(std_car_dist, Const.SONAR_STD, observedDist)
        prior_belief.normalize()
        # self.it-=1
            # if(self.it==0):
            #     print("done")

        new_belief = prior_belief
        new_belief.normalize()

        agg_row_col = random.choices([i for i in range(self.belief.numRows * self.belief.numCols)], weights =flatten(new_belief.grid), k = no_of_particles)


        for i in range(no_of_particles):
            particle_list[i] = (agg_row_col[i] // self.belief.numCols, agg_row_col[i] % self.belief.numCols)
        
        #update the belief using the particle
        self.belief = util.Belief(self.belief.numRows, self.belief.numCols, 0.0)
        for particle in particle_list:
            self.belief.addProb(particle[0], particle[1], 1.0)
        self.belief.normalize()

        return


    def estimate(self, posX: float, posY: float, observedDist: float, isParked: bool) -> None:
        # BEGIN_YOUR_CODE
        # implement Particle Filtering
        self.particle_filtering(posX, posY, observedDist, isParked, self.particle_no)
        # self.exact_inference(posX, posY, observedDist, isParked)
        # END_YOUR_CODE
        return
  
    def getBelief(self) -> Belief:
        return self.belief


    