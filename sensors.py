import pygame 
import math 
import numpy as np  

# calc unceratinty
def uncertainty_add(distance, angle, sigma):
    mean = np.array([distance, angle])
    covar = np.diag(sigma**2)
    distance, angle = np.random.multivariate_normal(mean, covar)
    distance = max(distance, 0)
    angle = max(angle, 0)
    return [distance, angle]

class LaserSensors: 

    # Constructor
    def __init__(self, range, map, uncertainty, num_points):
        self.range = range
        self.speed = 4
        self.sigma = np.array([uncertainty[0], uncertainty[1]])
        self.position = (0,0)
        self.map = map
        self.W, self.H = pygame.display.get_surface().get_size()
        self.sensedObstacles = []
        self.num_points = num_points

    #Helper to calc disrtance between two points
    def distance(self, obstaclePos):
        return math.sqrt((self.position[0]-obstaclePos[0])**2 + (self.position[1]-obstaclePos[1])**2)
    
    def senseObstacles(self): 
        data = []
        x1, y1 = self.position[0], self.position[1]
        for angle in np.linspace(0, 2*np.pi, self.num_points):
            x2, y2 = x1 + self.range * np.cos(angle), y1 + self.range * np.sin(angle)
            for i in range(0, 20):
                x = int(x1 + (x2-x1)*i/100)
                y = int(y1 + (y2-y1)*i/100)
                if 0< x < self.W and 0 < y < self.H:  
                    color = self.map.get_at((x,y))
                    if (color[0], color[1], color[2]) == (0,0,0):
                        distance = self.distance((x,y))
                        output = uncertainty_add(distance, angle, self.sigma)
                        output.append(self.position)
                        
                        # Add to the list of sensed obstacles
                        data.append(output)
                        break

        if len(data) > 0:
            return data
        else: return  False 


class Ultrasonic: 
    def __init__(self, sensor_range, map): 
        self.sensor_range = sensor_range
        self.map_width, self.map_height = pygame.display.get_surface().get_size()   
        self.map = map
    
    def sense_obstacles(self, x, y, heading): 
        obstacles = [] 
        x1, y1 = x, y
        start_angle = heading - self.sensor_range[0]
        end_angle = heading + self.sensor_range[0]

        for angle in np.linspace(start_angle, end_angle, 10 ,False): 
            x2, y2 = x1 + self.sensor_range[0]* np.cos(angle), y1 + self.sensor_range[0] * np.sin(angle)
            for i in range(0, 100):
                u = i / 100 
                x = int(x2 * u + x1 * (1 - u))
                y  = int(y2 * u + y1 * (1 - u))
                if 0 < x < self.map_width and 0 < y < self.map_height: 
                    color = self.map.get_at((x, y))
                    self.map.set_at((x, y), (0, 208, 255))
                    if (color[0], color[1], color[2]) == (0, 0, 0): 
                        obstacles.append((x, y))
                        break
        return obstacles 