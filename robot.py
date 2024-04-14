import math 
import numpy as np 
import pygame
import time

M2P = 3779.5275591 # meters to pixels
class Robot : 
    def __init__(self, start, width): 

        self.w = width 
        self.x = start[0]
        self.y = start[1]
        self.heading = 0
        self.vl = 0.01*M2P
        self.vr = 0.01*M2P 

        self.maxspeed = 0.02*M2P
        self.minspeed = 0.01*M2P

        self.min_obs_dist = 50
        self.count_down = 1
        self.segList = []

        self.dir_change = False


    def avoid_wall(self, seg_start, seg_end, dt): 
        self.segList.append((seg_start, seg_end))
        closest_obs = ((np.inf, np.inf), np.inf)
        heading_diff = 0
        facing_wall = False
        obs = self.distance_point_to_segment(seg_start, seg_end)
        if obs[1] < closest_obs[1]: 
            closest_obs = obs
            facing_wall, heading_diff = self.angle_point_to_segment(obs[0])   
        
        if abs(closest_obs[1]) < self.min_obs_dist:
            print("Avoiding wall that is ", closest_obs[1], " away", facing_wall, heading_diff)
            self.count_down -= dt
            print(self.vl, self.vr)
            if facing_wall: 
                time.sleep(0.01)
                self.rotate()
                self.dir_change = True
        else: 
            print("going forward, wall is ", closest_obs[1], "away")
            self.forward()
    
    def direction_change(self):
        self.vl = -self.vl
        self.vr = -self.vr

    def forward(self): 
        self.vl = self.minspeed
        self.vr = self.minspeed
    

    def backward(self):
        self.vl = -self.minspeed
        self.vr = -self.minspeed


#TODO fix the rotated function so it doesn't rotate robot into a wall
    def rotate(self): 
        self.heading -= np.pi/4



    def mid_pt(self, coord1, coord2):
        x1, y1 = coord1
        x2, y2 = coord2
        midpoint_x = (x1 + x2) / 2
        midpoint_y = (y1 + y2) / 2
        return (midpoint_x, midpoint_y)

       

    def kinematics(self, dt):  
        self.x += (self.vl + self.vr)/2 * math.cos(self.heading) * dt
        self.y -= (self.vl + self.vr)/2 * math.sin(self.heading) * dt #inverted comp screen 
        self.heading += (self.vr - self.vl)/self.w * dt

        if self.heading>2*math.pi:
            self.heading -= 2*math.pi
        elif self.heading<-2*math.pi:
            self.heading += 2*math.pi


    def angle_point_to_segment(self, point): 
        x, y = point
        angle = math.atan2(self.y - y, x - self.x)
        heading_diff = min(abs(angle - self.heading), 2*math.pi - abs(angle - self.heading))
        if heading_diff  < math.pi/3: 
            return True, heading_diff
        else: 
            return False, heading_diff


    def distance_point_to_segment(self, segment_start, segment_end):
        x, y = self.x, self.y
        x1, y1 = segment_start
        x2, y2 = segment_end

        dx = x2 - x1
        dy = y2 - y1

        if dx == 0 and dy == 0:
            # If the segment is just a point, return the distance between the point and the segment start
            return np.sqrt((x - x1) ** 2 + (y - y1) ** 2), segment_start

        t = ((x - x1) * dx + (y - y1) * dy) / (dx ** 2 + dy ** 2)
        t = np.clip(t, 0, 1)  # Ensure the intersection point is within the segment
        closest_x = x1 + t * dx
        closest_y = y1 + t * dy

        distance = np.sqrt((x - closest_x) ** 2 + (y - closest_y) ** 2)
        closest_point = (closest_x, closest_y)
        return closest_point, distance
        
class Graphics: 
    def __init__ (self, map_img_png, robot_img_path, robot_width, robot_height): 
        pygame.init()
        self.color = (0, 0, 0)
        self.robot = pygame.transform.scale(pygame.image.load(robot_img_path), (robot_width, robot_height))
        self.map = pygame.image.load(map_img_png)

    def draw_robot(self, x, y, heading): 
        rotated = pygame.transform.rotozoom(self.robot, math.degrees(heading), 1)
        rect = rotated.get_rect(center=(x, y))
        return rotated, rect 

        