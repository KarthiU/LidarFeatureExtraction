import numpy as np 
import math 
from fractions import Fraction 
from scipy.odr import * 

class featureDetection: 
    def __init__(self): 
        self.DELTA = 10
        self.EPISLON = 3 #ditsance from fitting line threshold
        self.SNUM = 6 #number of laser points in a seed segment 
        self.PMIN = 20 #min laser points to create a segment 
        self.GMAX = 5 #distancne between two points in segment
        self.SEED_SEGMENTS = []
        self.LINE_SEGMENTS = [] 
        self.LASERPOINTS = []
        self.LINE_PARAMS = None 
        self.NP = len(self.LASERPOINTS) - 1
        self.LMIN = 20 #min length of segment 
        self.LR = 0 # real length of segment
        self.PR = 0 # number of laser points contained in segment 

    def dist_p2p(self, p1, p2): 
        return math.sqrt((p1[0]-p2[0])**2 + (p1[1]-p2[1])**2)
    
    def dist_p2l(self, lineParams, p): 
        a, b, c = lineParams #coeffecients of the line (sstandard forrm)
        dist = abs(a*p[0] + b*p[1] + c)/math.sqrt(a**2 + b**2)
        return dist 
    
    #returns to points from slope intercept form depending on line 
    def line_to_coords(self, m, b): 
        x = 5 
        y = m * x + b 
        x2 = 100
        y2= m * x2 + b 
        return [(x, y), (x2, y2)] 
    
    def line_form_gen2SI(self, A, B, C): 
        m = -A/B
        b = -C/B
        return m, b
    
    def line_form_SI2gen(self, m, b):
        A = -m
        B = 1
        C = -b

        if A < 0: 
            A = -A 
            B = -B 
            C = -C

        den_a = Fraction(A).limit_denominator(1000).as_integer_ratio()[1]
        den_c = Fraction(C).limit_denominator(1000).as_integer_ratio()[1]

        gcd = np.gcd(den_a, den_c)
        lcm = den_a * den_c / gcd
        A = A * lcm
        B = B * lcm
        C = C * lcm

        return A, B, C
    
    #assumes lines intersect
    def line_intersect_general(self, l1params, l2params): 
        A1, B1, C1 = l1params
        A2, B2, C2 = l2params
        x = (B2*C1 - B1*C2)/(B1*A2 - A1*B2)
        y = (A1*C2 - A2*C1)/(A2*B1 - A1*B2)
        return x, y
    
    def points_2_slope_int(self, p1, p2): 
        m, b = 0, 0
        #undefined m
        if p2[0] == p1[0]: 
           pass
        else:
            m = (p2[1] - p1[1])/(p2[0] - p1[0])
            b = p2[1] - m*p2[0]
        return m, b
    
    def projections_p2line(self, p, m, b):
        x, y = p
        if abs(m) < 1e-10:  # Check if slope is close to zero (horizontal line)
            x_int = x
            y_int = b  # The y-intercept of the line
        else:
            m2 = -1 / m  # Slope of perpendicular line
            b2 = y - m2 * x  # y-intercept of perpendicular line
            x_int = - (b - b2) / (m - m2)  # x-coordinate of intersection
            y_int = m2 * x_int + b2  # y-coordinate of intersection
        return x_int, y_int

    def AD2pos(self, distance, angle, robotPosition): 
        x = distance * math.cos(angle) + robotPosition[0]
        y = distance * math.sin(angle) + robotPosition[1]
        return (int(x), int(y))
    
    def laser_points_set(self, data): 
        self.LASERPOINTS = []
        if not data: 
            pass 
        else: 
            for point in data: 
                coordinates = self.AD2pos(point[0], point[1], point[2])
                self.LASERPOINTS.append([coordinates, point[1]])
        self.NP = len(self.LASERPOINTS) - 1 

    def linear_func(self, p, x): 
        m, b = p 
        return m*x + b
    
    #orthognal distance regression (scipy odrr)
    def odr_fit(self, laser_points): 
        x = np.array([i[0][0] for i in laser_points])
        y = np.array([i[0][1] for i in laser_points])

        #Crerate a fittin gmodel 
        linear_model = Model(self.linear_func)

        #create a realData obj using initatined daa 
        data = RealData(x, y)

        #set up odr with model and data
        odr_model = ODR(data, linear_model, beta0=[0., 0.])

        out = odr_model.run() 
        m, b = out.beta
        return m, b

    def predictPoint(self, lineParams, sensedPoint, robotPos): 
        m, b = self.points_2_slope_int(robotPos, sensedPoint)
        params1 = self.line_form_SI2gen(m, b)
        predx, predy = self.line_intersect_general(params1, lineParams)
        return predx, predy

     
    def seed_segment_detection(self, robotPos, breakPointInd): 
        """
        Function to detect the seed segment
        params: robotPos: The current pos of the robot
                breakPointInd: The index of the break point
        """
        flag = True
        self.NP = max(0, self.NP) #so we don't get negative NP 
        self.SEED_SEGMENTS = [] 
        for i in range (breakPointInd, (self.NP - self.PMIN)): 
            predicted_pts_to_draw = []
            j = i + self.SNUM 
            m, c = self.odr_fit(self.LASERPOINTS[i:j])
            lineParams = self.line_form_SI2gen(m, c)
            
            for k in range(i, j): 
                if k == i: 
                    print("self.LASERPOINTS[k]: ", self.LASERPOINTS[k])
                pred_pt = self.predictPoint(lineParams, self.LASERPOINTS[k][0], robotPos)
                predicted_pts_to_draw.append(pred_pt)
                p2p_dist =  self.dist_p2p(pred_pt, self.LASERPOINTS[k][0])

                if p2p_dist > self.DELTA: 
                    flag = False 
                    break
                
                p2l_dist = self.dist_p2l(lineParams, self.LASERPOINTS[k][0])

                if p2l_dist > self.EPISLON: 
                    flag = False 
                    break

            if flag: 
                self.LINE_PARAMS = lineParams
                return [self.LASERPOINTS[i:j], predicted_pts_to_draw, (i, j)]
        return False
            

            
    def seed_segment_growing(self, indices, break_point):
        """
        Function to grow the seed segment
        param indices: The indices of the seed segment
        param break_point: The break point of the seed segment
        return: The seed segment, the two points, the line segment, the end point, the line equation, the line parameters
        """
        line_eq = self.LINE_PARAMS
        i, j = indices
        NP = len(self.LASERPOINTS)
        PB, PF = max(break_point, (i - 1) % NP), (j + 1) % NP

        while self.dist_p2l(line_eq, self.LASERPOINTS[PF][0]) < self.EPISLON:
            m, b = self.odr_fit(self.LASERPOINTS[PB:PF] if PB < PF else self.LASERPOINTS[PB:] + self.LASERPOINTS[:PF])
            line_eq = self.line_form_SI2gen(m, b)
            POINT = self.LASERPOINTS[PF][0]
            PF = (PF + 1) % NP
            NEXTPOINT = self.LASERPOINTS[PF][0]
            if self.dist_p2p(POINT, NEXTPOINT) > self.GMAX:
                break

        PF = (PF - 1) % NP

        while self.dist_p2l(line_eq, self.LASERPOINTS[PB][0]) < self.EPISLON:
            m, b = self.odr_fit(self.LASERPOINTS[PB:PF] if PB < PF else self.LASERPOINTS[PB:] + self.LASERPOINTS[:PF])
            line_eq = self.line_form_SI2gen(m, b)
            POINT = self.LASERPOINTS[PB][0]
            PB = (PB - 1) % NP
            NEXTPOINT = self.LASERPOINTS[PB][0]
            if self.dist_p2p(POINT, NEXTPOINT) > self.GMAX:
                break

        PB = (PB + 1) % NP

        LR = self.dist_p2p(self.LASERPOINTS[PB][0], self.LASERPOINTS[PF][0])
        PR = len(self.LASERPOINTS[PB:PF] if PB < PF else self.LASERPOINTS[PB:] + self.LASERPOINTS[:PF])
        if (LR >= self.LMIN) and (PR >= self.PMIN):
            self.LINE_PARAMS = line_eq
            m, b = self.line_form_gen2SI(line_eq[0], line_eq[1], line_eq[2])
            self.two_points = self.line_to_coords(m, b)
            self.LINE_SEGMENTS.append((self.LASERPOINTS[(PB + 1) % NP][0], self.LASERPOINTS[(PF - 1) % NP][0]))
            return [self.LASERPOINTS[PB:PF] if PB < PF else self.LASERPOINTS[PB:] + self.LASERPOINTS[:PF],
                    self.two_points, (self.LASERPOINTS[(PB + 1) % NP][0], self.LASERPOINTS[(PF - 1) % NP][0]),
                    PF, line_eq, (m, b)]
        else:
            return False
        


    def calculate_endpoints(self, idx): 
       outermost = self.LINE_SEGMENTS[idx][2]
       m, c = self.LINE_SEGMENTS[idx][5]
       return ( self.projections_p2line(outermost[0], m, c)
                ,self.projections_p2line(outermost[1],m, c))
    
    #TODO: Implement this function
    def overlap_region_processing(self, break_point): 
        overlap_fix_arr = []
        start, end =0,0
        j = 1
        for i in range(break_point, len(self.LINE_SEGMENTS) - 1): 
            EP_1 = self.calculate_endpoints(i)
            EP_2 = self.calculate_endpoints(j)

            LASERPOINTS_1 = self.LINE_SEGMENTS[i][0]
            LASERPOINTS_2 = self.LINE_SEGMENTS[j][0]

            if EP_1[1] >= EP_2[0]: 
                for point in LASERPOINTS_1: 
                    if False: 
                        pass
            else: 
                (start, end) = (EP_1[0], EP_2[1])

