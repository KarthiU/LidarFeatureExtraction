import math 
import pygame 

class BuildEnvironment: 
    def __init__(self, mapDim):
        pygame.init()
        self.pointCloud=[]
        self.externalMap = pygame.image.load('test.png')
        self.maph, self.mapw = mapDim 
        self.mapWindow = "Path Planning"
        pygame.display.set_caption(self.mapWindow)
        self.map = pygame.display.set_mode((self.mapw, self.maph))
        self.map.blit(self.externalMap, (0,0))

        #colors 
        self.white = (255,255,255)
        self.red = (255,0,0)
        self.green = (0,255,0)
        self.blue = (0,0,255)
        self.black = (0,0,0)
        self.grey = (100,100,100)


    def angleDist2Pos (self, distance, angle, robotPosition): 
        x = distance * math.cos(angle) + robotPosition[0]
        y = distance * math.sin(angle) + robotPosition[1]
        return (int(x), int(y))

    def dataStorage(self, data): 
        if data == False: 
          #  print("No obstacles detected in the sensor range.")
            return 
       # print(len(self.pointCloud))
        for element in data: 
            point = self.angleDist2Pos(element[0], element[1], element[2])
            if point not in self.pointCloud:
                self.pointCloud.append(point)

    def showSensorData(self): 
        self.infomap = self.map.copy()
        for point in self.pointCloud: 
            self.infomap.set_at((int(point[0]), int(point[1])), self.grey)


        
