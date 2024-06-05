class LaneFollower():
    def __init__(self, width, height, max_steer, normal_throttle, k_o, k_c):
        self.width = width
        self.height = height
        self.max_steer = max_steer
        self.normal_throttle = normal_throttle

        self.k_o = k_o
        self.k_c = k_c

        self.lane_offset = 0.0
        self.lane_curvature = 0.0
        
        self.throttle = 0.0
        self.steer = 0.0

    def calculate_control(self, middle_points):
        sum_x = 0
        sum_y = 0
        sum_x2 = 0
        sum_xy = 0
        n = 0
        for point in middle_points:
            if point is None:
                continue
            x, y = point
            y = y
            sum_x += x
            sum_y += y
            sum_x2 += x**2
            sum_xy += x * y
            n += 1

        a = 0
        b = 0
        if n > 1:
            if (n * sum_x2 - sum_x**2) == 0:
                return
            a = (n * sum_xy - sum_x * sum_y) / (n * sum_x2 - sum_x**2)
            b = (sum_y * sum_x2 - sum_x * sum_xy) / (n * sum_x2 - sum_x**2)
            if a == 0:
                return
            lane_start_point = (self.height - b) / a
            lane_middle_point = (self.height/2 - b) / a
            self.lane_offset = (lane_start_point - (self.width/2)) / (self.width/2)
            self.lane_curvature = (lane_middle_point - lane_start_point) / (self.width/2)
        else:
            return
        
        if 1 < self.lane_offset or 1 < self.lane_curvature:
            print("Range ERROR")
            return
        
        # print(f'self.lane_offset = {self.lane_offset}')
        # print(f'self.lane_curvature = {self.lane_curvature}')

        self.steer = self.k_o * self.lane_offset + self.k_c * self.lane_curvature
        self.steer *= self.max_steer 
        
        if self.max_steer < self.steer:
            self.steer = self.max_steer
        elif self.steer < -self.max_steer:
            self.steer = -self.max_steer
            
        self.throttle = self.normal_throttle
        
        # print(f'self.steer = {self.steer}')
        return
    
    def get_control(self):
        return self.throttle, self.steer