import cv2
import numpy as np
import math
import board
import busio
import adafruit_mcp4728
import time

# constants
stop_car = 65000 // 2
start_car = 37000
# PD consts
P_VAL = 1
D_VAL = 0.1

# Useful variables
lastTime = 0 
lastError = 0
tickCount = 0
stopSignsReached = 0 # num stop signs
currentSpeed = start_car
# store data for plots
pValues = open("p_values.txt", 'a')
dValues = open("d_values.txt", 'a')
steeringValues = open("steering_values.txt", 'a')
throttleValues = open("throttle_values.txt", 'a')


def getRedFloorBoundaries():
    """
    Gets the hsv boundaries and success boundaries indicating if the floor is red
    :return: [[lower color and success boundaries for red floor], [upper color and success boundaries for red floor]]
    """
    return getBoundaries("redboundaries.txt")

def isRedFloorVisible(frame):
    """
    Detects whether or not the floor is red
    :param frame: Image
    :return: [(True is the camera sees a red on the floor, false otherwise), video output]
    """
    #print("Checking for floor stop")
    boundaries = getRedFloorBoundaries()
    return isMostlyColor(frame, boundaries)

def isMostlyColor(image, boundaries):
    """
    Detects whether or not the majority of a color on the screen is a particular color
    :param image:
    :param boundaries: [[color boundaries], [success boundaries]]
    :return: boolean if image satisfies provided boundaries, and an image used for debugging
    """
    #Convert to HSV color space
    hsv_img = cv2.cvtColor(image, cv2.COLOR_BGR2HSV)

    #parse out the color boundaries and the success boundaries
    color_boundaries = boundaries[0]
    percentage = boundaries[1]
    
    lower = np.array(color_boundaries[0])
    # print("lower:" + str(lower))
    upper = np.array(color_boundaries[1])
    mask = cv2.inRange(hsv_img, lower, upper)
    output = cv2.bitwise_and(hsv_img, hsv_img, mask=mask)

    #Calculate what percentage of image falls between color boundaries
    percentage_detected = np.count_nonzero(mask) * 100 / np.size(mask)
    # print("percentage_detected " + str(percentage_detected) + " lower " + str(lower) + " upper " + str(upper))
    # If the percentage percentage_detected is betweeen the success boundaries, we return true, otherwise false for result
    result = percentage[0] < percentage_detected <= percentage[1]
    if result:
        pass
    return result, output

def getBoundaries(filename):
    """
    Reads the boundaries from the file filename
    Format:
        [0] lower: [H, S, V, lower percentage for classification of success]
        [1] upper: [H, S, V, upper percentage for classification of success]
    :param filename: file containing boundary information as above
    :return: [[lower color and success boundaries], [upper color and success boundaries]]
    """
    default_lower_percent = 50
    default_upper_percent = 100
    with open(filename, "r") as f:
        boundaries = f.readlines()
        lower_data = [val for val in boundaries[0].split(",")]
        upper_data = [val for val in boundaries[1].split(",")]

        if len(lower_data) >= 4:
            lower_percent = float(lower_data[3])
        else:
            lower_percent = default_lower_percent

        if len(upper_data) >= 4:
            upper_percent = float(upper_data[3])
        else:
            upper_percent = default_upper_percent

        lower = [int(x) for x in lower_data[:3]]
        upper = [int(x) for x in upper_data[:3]]
        boundaries = [lower, upper]
        percentages = [lower_percent, upper_percent]
    return boundaries, percentages

def convert_to_HSV(frame):
    hsv = cv2.cvtColor(frame, cv2.COLOR_BGR2HSV)
    cv2.imshow("HSV",hsv)
    return hsv

def detect_edges(frame):
    lower_blue = np.array([90, 120, 0], dtype = "uint8") # lower limit of blue color
    upper_blue = np.array([150, 255, 255], dtype="uint8") # upper limit of blue color
    mask = cv2.inRange(hsv,lower_blue,upper_blue) # this mask will filter out everything but blue

    # detect edges
    edges = cv2.Canny(mask, 50, 100) 
    cv2.imshow("edges",edges)
    return edges

def region_of_interest(edges):
    height, width = edges.shape # extract the height and width of the edges frame
    mask = np.zeros_like(edges) # make an empty matrix with same dimensions of the edges frame

    # only focus lower half of the screen
    # specify the coordinates of 4 points (lower left, upper left, upper right, lower right)
    polygon = np.array([[
        (0, height), 
        (0,  height/2),
        (width , height/2),
        (width , height),
    ]], np.int32)

    cv2.fillPoly(mask, polygon, 255) # fill the polygon with blue color 
    cropped_edges = cv2.bitwise_and(edges, mask) 
    cv2.imshow("roi",cropped_edges)
    return cropped_edges

def detect_line_segments(cropped_edges):
    rho = 1  
    theta = np.pi / 180  
    min_threshold = 10 
    line_segments = cv2.HoughLinesP(cropped_edges, rho, theta, min_threshold, 
                                    np.array([]), minLineLength=5, maxLineGap=0)
    return line_segments

def average_slope_intercept(frame, line_segments):
    lane_lines = []

    if line_segments is None:
        print("no line segment detected")
        return lane_lines

    height, width,_ = frame.shape
    left_fit = []
    right_fit = []
    boundary = 1/3

    left_region_boundary = width * (1 - boundary) 
    right_region_boundary = width * boundary 

    for line_segment in line_segments:
        for x1, y1, x2, y2 in line_segment:
            if x1 == x2:
                print("skipping vertical lines (slope = infinity)")
                continue

            fit = np.polyfit((x1, x2), (y1, y2), 1)
            slope = (y2 - y1) / (x2 - x1)
            intercept = y1 - (slope * x1)

            if slope < 0:
                if x1 < left_region_boundary and x2 < left_region_boundary:
                    left_fit.append((slope, intercept))
            else:
                if x1 > right_region_boundary and x2 > right_region_boundary:
                    right_fit.append((slope, intercept))

    left_fit_average = np.average(left_fit, axis=0)
    if len(left_fit) > 0:
        lane_lines.append(make_points(frame, left_fit_average))

    right_fit_average = np.average(right_fit, axis=0)
    if len(right_fit) > 0:
        lane_lines.append(make_points(frame, right_fit_average))

    # lane_lines is a 2-D array consisting the coordinates of the right and left lane lines
    # for example: lane_lines = [[x1,y1,x2,y2],[x1,y1,x2,y2]]
    # where the left array is for left lane and the right array is for right lane 
    # all coordinate points are in pixels
    return lane_lines

def make_points(frame, line):
    height, width, _ = frame.shape
    slope, intercept = line
    y1 = height  # bottom of the frame
    y2 = int(y1 / 2)  # make points from middle of the frame down

    if slope == 0: 
        slope = 0.1    

    x1 = int((y1 - intercept) / slope)
    x2 = int((y2 - intercept) / slope)

    return [[x1, y1, x2, y2]]

def display_lines(frame, lines, line_color=(0, 255, 0), line_width=6): # line color (B,G,R)
    line_image = np.zeros_like(frame)

    if lines is not None:
        for line in lines:
            for x1, y1, x2, y2 in line:
                cv2.line(line_image, (x1, y1), (x2, y2), line_color, line_width)

    line_image = cv2.addWeighted(frame, 0.8, line_image, 1, 1)  
    return line_image

def get_steering_angle(frame, lane_lines):
    height, width, _ = frame.shape

    if len(lane_lines) == 2: # if two lane lines are detected
        _, _, left_x2, _ = lane_lines[0][0] # extract left x2 from lane_lines array
        _, _, right_x2, _ = lane_lines[1][0] # extract right x2 from lane_lines array
        mid = int(width / 2)
        x_offset = (left_x2 + right_x2) / 2 - mid
        y_offset = int(height / 2)  
    elif len(lane_lines) == 1: # if only one line is detected
        x1, _, x2, _ = lane_lines[0][0]
        x_offset = x2 - x1
        y_offset = int(height / 2)
    elif len(lane_lines) == 0: # if no line is detected
        x_offset = 0
        y_offset = int(height / 2)

    angle_to_mid_radian = math.atan(x_offset / y_offset)
    angle_to_mid_deg = int(angle_to_mid_radian * 180.0 / math.pi)  
    steering_angle = angle_to_mid_deg + 90 

    return steering_angle

def display_heading_line(frame, steering_angle, line_color=(0, 0, 255), line_width=5 ):

    heading_image = np.zeros_like(frame)
    height, width, _ = frame.shape

    steering_angle_radian = steering_angle / 180.0 * math.pi
    x1 = int(width / 2)
    y1 = height
    x2 = int(x1 - height / 2 / math.tan(steering_angle_radian))
    y2 = int(height / 2)

    cv2.line(heading_image, (x1, y1), (x2, y2), line_color, line_width)

    heading_image = cv2.addWeighted(frame, 0.8, heading_image, 1, 1)

    return heading_image

def deviation_to_command(error, lastTime, lastError, dValues, pValues):
    PID_output = 0
    # if deviation < 5 and deviation > -5: # do not steer if there is a 10-degree error range
    #     PID_output = 0
    now = time.time() # current time variable
    dt = now - lastTime
    deviation = steering_angle - 90 # equivalent to angle_to_mid_deg variable 

    # Get P and D values and figure out the actual command
    derivative = D_VAL * (error - lastError) / dt 
    proportional = P_VAL * error
    dValues.write(str(derivative) + '\n')
    pValues.write(str(proportional) + '\n')

    # Actual command
    PID_output = int(derivative + proportional)

    # Next values
    lastError = error
    lastTime = time.time()
    print(PID_output)
    return [PID_output, lastError, lastTime]

def adjust_steering(control_val, mcp4728, steeringValues):
    print("Input angle = " + str(control_val))
    # Bound between -45 and 45
    if control_val < -45: control_val = -45
    elif control_val > 45: control_val = 45
    center = 65000 / 2
    # Get the steering input
    input = int(center + control_val * (32500 / 45))
    print("Controller steering input = " + str(input))
    steeringValues.write(str(input) + '\n')
    # Set the steering input
    mcp4728.channel_a.value = input


def increase_speed(currentSpeed, throttleValues):
    # currentSpeed += 100
    if (currentSpeed > 65535): currentSpeed = 65535
    mcp4728.channel_c.value = currentSpeed
    print("Increased speed! curr val is " + str(currentSpeed))
    throttleValues.write(str(currentSpeed) + '\n')
    return currentSpeed

# def decrease_speed(currentSpeed, throttleValues):
#     # currentSpeed -= 100
#     if (currentSpeed < 0): currentSpeed = 0
#     mcp4728.channel_c.value = currentSpeed
#     print("Decreased speed... curr val is " + str(currentSpeed))
#     throttleValues.write(str(currentSpeed) + '\n')
#     return currentSpeed

#*********************** MAIN CODE ***********************
try:
    # Set up video interface
    video = cv2.VideoCapture('/dev/video0')
    video.set(cv2.CAP_PROP_FRAME_WIDTH,320)
    video.set(cv2.CAP_PROP_FRAME_HEIGHT,240)

    # Set up I2C interface with controller
    i2c = busio.I2C(board.SCL, board.SDA)
    mcp4728 = adafruit_mcp4728.MCP4728(i2c, 0x64)

    mcp4728.channel_c.value = start_car
    throttleValues.write(str(start_car) + '\n')

    while True:
        # Get the frame from the camera
        ret,original_frame = video.read()
        frame = cv2.resize(original_frame, (160, 120))
        frame = cv2.flip(frame,-1)

        # Check the current speed off encoder
        with open("/sys/module/speed_driver2/parameters/elapsedTime", "r") as filetoread:
            time_diff = int(filetoread.read())

        print("Time diff", time_diff)
        # Adjust accordingly
        # if time_diff >= 120:
        currentSpeed = increase_speed(currentSpeed, throttleValues)
        # elif time_diff <= 110 and time_diff > 7:
        #     currentSpeed = decrease_speed(currentSpeed, throttleValues)

        # Locate approximate area where track is
        hsv = convert_to_HSV(frame)
        edges = detect_edges(hsv)
        roi = region_of_interest(edges)

        # Identify guiding edges
        line_segments = detect_line_segments(roi)
        lane_lines = average_slope_intercept(frame,line_segments)

        # Extrapolate the required angle correction
        steering_angle = get_steering_angle(frame, lane_lines)
        deviation = steering_angle - 90

        # Convert into actual control command and store results
        ret = deviation_to_command(deviation, lastTime, lastError, dValues, pValues)
        adjust_steering(ret[0], mcp4728, steeringValues)
        # Store previous values
        lastTime = ret[1]
        lastError = ret[2]

        # Determine if there is a stop sign (every 5th loop)
        if (tickCount % 5 == 0):
            atStopSign, _ = isRedFloorVisible(frame)
            if (atStopSign):
                print("Stop sign detected!")
                # Stop the vehicle for 3s if first, forever if second
                if (stopSignsReached == 0):
                    # Handle first stop sign interaction
                    mcp4728.channel_c.value = stop_car
                    throttleValues.write(str(stop_car) + '\n')
                    time.sleep(3)
                    # Resume driving
                    mcp4728.channel_c.value = currentSpeed
                    throttleValues.write(str(currentSpeed) + '\n')
                    stopSignsReached += 1
                elif (stopSignsReached == 1):
                    # Second stop sign, terminate
                    mcp4728.channel_c.value = stop_car
                    throttleValues.write(str(stop_car) + '\n')
                    break
        tickCount += 1
        
except KeyboardInterrupt or Exception:
    # Handling for end to program
    print("Stopping program...")
    mcp4728.channel_c.value = stop_car
    throttleValues.write(str(stop_car) + '\n')
