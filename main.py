import env, sensors, features
import pygame 
import math
import random
from robot import Graphics, Robot

M2P = 3779.5275591 # meters to pixels
SCREEN_H = 600
SCREEN_W =1200

def random_color(): 
    levels = range(32,256,32)
    return tuple(random.choice(levels) for _ in range(3))

robot_img = pygame.image.load('robot.png')

gfx = Graphics('test.png', 'robot.png', 20, 20)



featureMap = features.featureDetection()
environment = env.BuildEnvironment((SCREEN_H,SCREEN_W))
originalMap = environment.map.copy()
laser = sensors.LaserSensors(500, originalMap,uncertainty=(0.5, 0.01), num_points=120)
environment.map.fill(environment.white)
environment.map.blit(gfx.map, (0,0))
environment.infomap = environment.map.copy()
originalMap = environment.map.copy()

running = True
FEATURE_DETECTION = True
BREAK_POINT_IND = 0 


start = (400, 200)
robot = Robot(start, 0.01*M2P)

dt = 0 
last_time = pygame.time.get_ticks()

# Create a robot objectwhile running: 
while running:
    #environment.infomap=originalMap.copy()
    FEATURE_DETECTION = True
    BREAK_POINT_IND = 0 
    ENDPOINTS = [(0,0), (0,0)]
    sensorOn = True 
    PREDICTED_PTS_TO_DRAW = []
    for event in pygame.event.get(): 
        if event.type == pygame.QUIT:
            running = False

    dt = (pygame.time.get_ticks() - last_time)/1000.0
    last_time = pygame.time.get_ticks()

    
    if sensorOn: 
        position = (robot.x, robot.y)
        laser.position = position
        sensor_data = laser.senseObstacles()
        environment.dataStorage(sensor_data)
        featureMap.laser_points_set(sensor_data)
        while BREAK_POINT_IND < (featureMap.NP - featureMap.PMIN):
            seedSeg = featureMap.seed_segment_detection(laser.position, BREAK_POINT_IND)
            if seedSeg == False: 
                break
            else: 
                seedSegment = seedSeg[0]
                PREDICTED_PTS_TO_DRAW = seedSeg[1]
                INDICES = seedSeg[2]
                results = featureMap.seed_segment_growing(INDICES, BREAK_POINT_IND)
                if results == False: 
                    BREAK_POINT_IND = INDICES[1]
                    continue 
                else: 
                    line_eq=results[1]
                    m, c = results[5]
                    line_seg = results[0]
                    OUTERMOST = results[2]
                    BREAK_POINT_IND = results[3]
                    ENDPOINTS[0] = featureMap.projections_p2line(OUTERMOST[0], m, c)
                    ENDPOINTS[1] = featureMap.projections_p2line(OUTERMOST[1], m, c)
                    COLOR = random_color()
                    #for point in line_seg: 
                        #environment.infomap.set_at((int(point[0][0]), int(point[0][1])), (0, 255, 0))
                        #pygame.draw.circle(environment.infomap, COLOR, (int(point[0][0]), int(point[0][1])), 2, 0)

                    environment.dataStorage(sensor_data)

                    if ENDPOINTS != [(0,0), (0,0)]: 
                        robot.avoid_wall(ENDPOINTS[0], ENDPOINTS[1], dt)
            
            
            pygame.draw.line(environment.infomap, (255, 0, 0), ENDPOINTS[0], ENDPOINTS[1], 2)

    environment.map.blit(environment.infomap, (0,0))
    # print("running")
    robot.kinematics(dt)
   
    rotated, rect = gfx.draw_robot(robot.x, robot.y, robot.heading)
    environment.map.blit(rotated, rect)
   # print(ENDPOINTS[0], ENDPOINTS[1])


    if robot.x < 200 or robot.x > SCREEN_W or robot.y < 25 or robot.y > SCREEN_H: 
        robot.x, robot.y = start
        robot.heading = 0

    pygame.display.update()
