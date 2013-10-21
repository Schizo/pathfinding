import heapq
import types
import time

print "========= starting script ==========="
# This code is called when instances of this SOP cook.
geo = hou.pwd().geometry()



#get attribute objects to make attrib lookup faster later.
atrG = geo.findPointAttrib("g")
atrF =  geo.findPointAttrib("f")
atrH =  geo.findPointAttrib("h")
atrN = geo.findPointAttrib("neighbour2")
atrID = geo.findPointAttrib("pid")
atrPARENT = geo.findPointAttrib("parent")

#get all of the points now.
allPoints = geo.points()
allPoints = list(allPoints)

#starting from the goal, find the neighbor with the lowest distance to trace back to the start. 
endPoint = geo.globPoints("goal")[0]
startPoint = geo.globPoints("start")[0]


class pointNode(object):
    """docstring for classname"""
    def __init__(self, x, y, z, reachable):
        """ptnum
        @param x cell x cooridnate
        @param y cell y cooridnate
        @param z cell z cooridnate
        """
        self.reachable = reachable
        self.x = x
        self.y = y
        self.parent = None
        self.g = 0
        self.h = 0
        self.f = 0

class Astar(object):
    """docstring for Astar"""
    def __init__(self):
        self.op = []
        heapq.heapify(self.op)
        self.cl = set()
        self.points = []

    def debugPoints(self):
        for i in self.points:
            print i.attribValue("neighbourCount")

    def displayPath(self):
        pointPos = self.end

        while pointPos.attribValue(atrID) is not self.start.attribValue(atrID): 
              point = geo.createPoint()
              point.setPosition(pointPos.position())
              pointPos = self.points[pointPos.attribValue(atrPARENT)]
              print pointPos.attribValue(atrID)
        #Add Starting Point as well to the pointslist
        point = geo.createPoint()
        point.setPosition(self.start.position())
    
            
    def initGrid(self, start, end, allPoints):
        self.start = start
        self.end = end
        self.points = allPoints

    def getNeighbourPoints(self, id):
        pointsList = []
        parent = 0
        
        #getNeighbour Count of current Point
        neighbours = self.points[id].attribValue("neighbourCount")
        #idValue = self.points[id].attribValue(idN)
 
       
        #collect all neighbours
        for x in range(neighbours):
           parent = self.points[id].attribValue("neighbour" + str(x+1))
           pointsList.append(self.points[parent])
        return pointsList 


    def getHeuristic(self, point):
        """
        Compute the heuristic H for a Point: distance between this Point and the ending Point multiply by 10.
        @param cell
        @returns heuristic value H
        """
        return 10 * hou.Vector3.distanceTo(point, self.end)

    def updatePoint(self, point, parent):
        distanceToGoal = point.position() - self.end.position()
        distanceToParent = point.position() - parent.position() #maybe we have to absol here
        #distanceToParent = pare #maybe we have to absol here
        point.setAttribValue("g", parent.attribValue(atrG) + hou.Vector3.length(distanceToParent)) #take length of vector as heuristic
        point.setAttribValue("h", hou.Vector3.length(distanceToGoal))
        point.setAttribValue("parent", parent.attribValue(atrID))
        point.setAttribValue ("f", point.attribValue(atrG) + point.attribValue(atrH) )
         

    def process(self):
        heapq.heappush(self.op, (self.start.attribValue(atrF), self.start))
        while len(self.op):
            #pop point from heap queue
            f, cell = heapq.heappop(self.op)
            #add cell to closed list so we don't process it twice
            self.cl.add(cell)
            #if ending cell display found path
            
            
            if cell is self.end:
                break
            #get neighbour cells
            
            neighbours =  self.getNeighbourPoints(cell.attribValue(atrID))
            for c in neighbours:
                if c not in self.cl:
                   if (c.attribValue(atrF), c) in self.op:   
                       if c.g > cell.g + 10:
                            self.updatePoint(c, cell)
                   else:
                        self.updatePoint(c, cell)
                        heapkey = str(c.attribValue(atrF))
                        heapq.heappush(self.op, (heapkey, c))




algorithm = Astar()
algorithm.initGrid(allPoints[68], allPoints[194], allPoints)
start = time.clock()
algorithm.process()
elapsed = (time.clock() - start)
print elapsed
algorithm.displayPath()


print "========= ending script ==========="
