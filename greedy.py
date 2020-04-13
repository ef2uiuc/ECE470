# -*- coding: utf-8 -*-
# Given only 3 bins and an array of X amount of materials and their costs,
# which bin should you put them in to minimize costs?
import sim
from getHandles import getHandles
from remote import clientID
noBins = 3
names = ['red', 'blue', 'green', 'purple', 'white','Empty']
costs = [100, 50, 20, 5, 200, 0]
sensor = getHandles('Proximity_sensor')

def material(names,costs):
    material = {}
    for i in range(len(names)):
        material[names[i]] = costs[i]
    return material

materials = material(names,costs)

class greedyAlgo:
    def __init__(self, materials, noBins):
        self.materials = materials
        self.bins = {}
        self.runningCost = 0
        for i in range(noBins):
            self.bins[i] = 'Empty'
            
    def update(self, inMat):
        #inMat should be the material being currently sorted
        #should find the first a bin of its kind
        #otherwise finds next empty bin
        #otherwise takes bin of lowest cost
        #then returns the bin it should enter
        curBins = list(self.bins.values())
        binToGo = 0
        for i, option in enumerate(curBins):
            if option == inMat:
                binToGo = i
                return binToGo
            if option == 'Empty':
                binToGo = i
                self.bins[binToGo] = inMat
                self.runningCost += self.materials[inMat]
                return binToGo
            else:
                costs = [self.materials[curBins[0]], self.materials[curBins[1]], self.materials[curBins[2]]]
                binToGo = costs.index(min(costs))
                self.bins[binToGo] = inMat
                self.runningCost += self.materials[inMat]
                return binToGo
        
        
        
        
