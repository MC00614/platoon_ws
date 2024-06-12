import numpy as np
import cv2 as cv
import math

class ImageProcessor():
    def __init__(self):
        self.hsv_values = (0, 0, 232) #(0, 0, 160)
        self.first_left_box_middle_x = 100      #Middlepoint of left Box
        self.first_right_box_middle_x = 540    #Middlepoint of right Box
        self.warp_parameters = [(190,220),(85 ,480),(430,220),(520,480)]    #Warping Parameters tl, bl, tr, br
        self.num_boxes = 3      #Number of boxes
        self.box_dim = (150, 100) # height, width
        self.half_lane_width = 270	#assumpt half lane in pixels
        # Hough Line Transform
        self.rho = 1			
        # Angle resolution of the accumulator in radians.
        self.theta = np.pi/180
        # Only lines that are greater than threshold will be returned.
        self.threshold = 50
        # Line segments shorter than that are rejected.
        self.minLineLength = 30
        # Maximum allowed gap between points on the same line to link them
        self.maxLineGap = 5

    def frame_processor(self, image):
        warped_image = self.warp_image(image)
        
        edges = cv.Canny(warped_image, 10, 30)
        # cv.imshow('Canny', edges)
        contours, _ = cv.findContours(edges, cv.RETR_EXTERNAL, cv.CHAIN_APPROX_SIMPLE)
        cv.drawContours(edges, contours, -1, (255), 5)
        # cv.imshow('edges', edges)

        #Using HSV filter
        frame_HSV = cv.cvtColor(warped_image, cv.COLOR_BGR2HSV)
        #values are tested in testing script "hsv_filter". the 3rd value can be ajusted between 150-200
        image_hsv = cv.inRange(frame_HSV, self.hsv_values, (180, 255, 255))
        # cv.imshow('hsv', image_hsv)
        
        add_image = cv.bitwise_and(edges, image_hsv)
        # cv.imshow('add_image', add_image)
        image_hsv = add_image

        #apply the sliding window for left and right lane with base midpoint of lane at xm
        left, left_line = self.sliding_windows(image_hsv, warped_image, xm=self.first_left_box_middle_x)
        right, right_line = self.sliding_windows(image_hsv, warped_image, xm=self.first_right_box_middle_x)

        #calculate middlepoints with left and right lane points 
        middle_points = self.calculate_middle_path(left, right)

        for point in middle_points:
            self.draw_lines_points(warped_image, point=point)
        return middle_points
    
    def warp_image(self, image):
        height, width, depth = image.shape
        #create warped image with fixes parameters
        #warp_parameters = [(320,110),(0 ,420),(800,110),(1070,420)]
        # tl = (320,110)
        # bl = (0 ,420)
        # tr = (800,110)
        # br = (1070,420)
        ## Aplying perspective transformation
        pts1 = np.float32([self.warp_parameters[0], self.warp_parameters[1], self.warp_parameters[2], self.warp_parameters[3]]) 
        pts2 = np.float32([[0, 0], [0, height], [width, 0], [width, height]]) 

        # Matrix to warp the image for birdseye window
        matrix = cv.getPerspectiveTransform(pts1, pts2) 
        transformed_frame = cv.warpPerspective(image, matrix, (width,height))
        #cv.imshow('warp', transformed_frame)
        return transformed_frame

    def sliding_windows(self, image, warped_image, xm=320):
        num_windows = self.num_boxes
        height, width = image.shape
        result = [[-1,-1,-1]]*num_windows
        line = [[-1,-1,-1]]*num_windows
        midpoint = (xm, height-self.box_dim[0])
        # im_h = self.sobel_inner_line(image, xm)
        # cv.imshow('Sobel Image', im_h)
        for i in range(num_windows):
            # masked_image = roi_boxes(im_h, midpoint)
            # cv.imshow('Masked Image', masked_image)
            # print(midpoint)
            # lines = hough_transform(masked_image)
            lines = self.roi_hough_transform(image, midpoint)
            if lines is not None:
                average_line, average_lane_line = self.average_lane_lines(lines, midpoint)
                if len(average_lane_line) <= 1:
                    break
                point = (average_lane_line[2].astype(int),midpoint[1])
                result[i] = point
                line[i] = average_lane_line
                self.draw_lines_points(warped_image, average_line, point)
                midpoint = (point[0], height-(i+2)*self.box_dim[0])
        return result, line
    
    def sobel_inner_line(self, image, xm):
        #apply sobel filter in x direction
        height, width = image.shape
        if xm <= width/2:	#for left lane invert the image
            img = cv.bitwise_not(image)
        else:
            img = image
        sobelx= cv.Sobel(img,cv.CV_8U,1,0,ksize=3) #only ditect inner lines
        return sobelx
    
    def roi_hough_transform(self, image, midpoint):
        #cv.imshow('ROI Image', image)
        height, width = image.shape
        box_height, box_width = self.box_dim       
        if midpoint[1] == height - box_height:
            box_width = 250
        start_point = (midpoint[0]-box_width, midpoint[1])
        end_point = (midpoint[0]+box_width, midpoint[1]+box_height)
        # function returns an array containing dimensions of straight lines 
        # appearing in the input image
        lines = cv.HoughLinesP(image[max(0,start_point[1]):min(720,end_point[1]), max(0,start_point[0]):min(1280,end_point[0])], rho = self.rho, theta = self.theta, threshold = self.threshold,
            minLineLength = self.minLineLength, maxLineGap = self.maxLineGap)
        if lines is not None:
            for i in range(len(lines)):
                lines[i][0][0] += max(0,start_point[0])
                lines[i][0][1] += max(0,start_point[1])
                lines[i][0][2] += max(0,start_point[0])
                lines[i][0][3] += max(0,start_point[1])
        return lines
    
    def average_lane_lines(self, lines, midpoint): 
        valid_right_line = []
        valid_lines = [] #(slope, intercept)
        xm,ym = midpoint
        if lines is not None:
            # Convert lines to a NumPy array if it's not already
            lines = np.array(lines)

            # Extract x1, y1, x2, y2 for all lines
            x1 = lines[:, :, 0]
            y1 = lines[:, :, 1]
            x2 = lines[:, :, 2]
            y2 = lines[:, :, 3]

            # Handle the cases where x1 == x2 or y1 == y2
            x2 = np.where(x1 == x2, x2 + 1, x2)

            # Calculate slopes
            slopes = (y2 - y1) / (x2 - x1)

            # Calculate intercepts
            intercepts = y1 - (slopes * x1)

            # Calculate x-intercepts with horizontal line ym
            x_intercepts = (ym - intercepts) / slopes

            # Filter out invalid lines (where y1 == y2)
            valid_indices = (y1 != y2) & (np.abs(slopes) >= 0.2) #& (x_intercepts < xm)

            valid_right_lines = lines[valid_indices]
            valid_slopes = slopes[valid_indices]
            valid_intercepts = intercepts[valid_indices]
            valid_x_intercepts = x_intercepts[valid_indices]

            # Prepare valid_lines as tuples of (slope, intercept, x)
            valid_lines_array = np.column_stack((valid_slopes, valid_intercepts, valid_x_intercepts))

            valid_right_line.extend(valid_right_lines.tolist())
            valid_lines.extend(valid_lines_array.tolist())

            # Calculate the average of valid lines
            if len(valid_lines) > 0:
                average_line = np.mean(valid_right_line, axis=0)
                average_lane_line = np.mean(valid_lines, axis=0)
                return average_line.astype(int), average_lane_line
            else:
                return [[-1, -1, -1, -1]], [[-1, -1, -1]]

    def draw_lines_points(self, image, lines=None, point=None):
        #draw lines and points and show the image
        draw_image = image

        if lines is not None:
            # Convert lines to a NumPy array if it's not already
            lines = np.array(lines)

            if lines.ndim == 1:
                lines = lines.reshape(1, 4)

            # Extract x1, y1, x2, y2 for all lines
            x1 = lines[:, 0]
            y1 = lines[:, 1]
            x2 = lines[:, 2]
            y2 = lines[:, 3]
            
            # Iterate through lines using NumPy arrays
            for i in range(len(lines)):
                cv.line(draw_image, (int(x1[i]), int(y1[i])), (int(x2[i]), int(y2[i])), (0, 0, 255), 5)

        if point is not None:
            cv.circle(draw_image, (point[0], point[1]), radius=5, color=(0, 255, 0), thickness=-1)

        # cv.imshow("Hough Transformation", draw_image)

    def calculate_middle_path(self, left, right):
        #calculate the middle of left and right lane with given parameters
        middle = [None] * self.num_boxes
        for i in range(len(left)):
            x_left = left[i]
            x_right = right[i]
            if len(x_left) <=2 and len(x_right) <=2:
                #in case of both lanes detected, calculate middle 
                middle[i] = (int((x_right[0]/2 + x_left[0]/2)),x_left[1])
            elif x_left != [-1,-1,-1]:
                #if only left lane is detected, calculate middle 
                middle[i] = (x_left[0]+self.half_lane_width,x_left[1])
            elif x_right != [-1,-1,-1]:
                #if only right lane is detected, calculate middle 
                middle[i] = (x_right[0]-self.half_lane_width,x_right[1])
        return middle
    