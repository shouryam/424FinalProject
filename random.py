"""
FACTOR = 20000000 / 100
throttlePin = "P8_13"
go_forward = 8.2
go_faster_addition = 0.01
go_faster_tick_delay = 80
go_faster_tick = 0  # Do not change this here. Code will set this value after seeing stop sign
dont_move = 7.5

# Steering
steeringPin = "P9_14"
left = 9
right = 6


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
    upper = np.array(color_boundaries[1])
    mask = cv2.inRange(hsv_img, lower, upper)
    output = cv2.bitwise_and(hsv_img, hsv_img, mask=mask)

    #Calculate what percentage of image falls between color boundaries
    percentage_detected = np.count_nonzero(mask) * 100 / np.size(hsv_img)
    #print("percentage_detected " + str(percentage_detected) + " lower " + str(lower) + " upper " + str(upper))
    # If the percentage percentage_detected is betweeen the success boundaries, we return true, otherwise false for result
    result = percentage[0] < percentage_detected <= percentage[1]
    if result:
        print("STOP PERCENTAGE: ", percentage_detected)
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

def initialize_car():
    # give 7.5% duty at 50Hz to throttle
    with open('/dev/bone/pwm/1/a/duty_cycle', 'w') as filetowrite:
        filetowrite.write(str(int(dont_move * FACTOR)))
    # wait for car to be ready
    input()

    with open('/dev/bone/pwm/1/b/duty_cycle', 'w') as filetowrite:
        filetowrite.write(str(int(dont_move * FACTOR)))

def stop():
    """
    Stops the car
    :return: none
    """
    with open('/dev/bone/pwm/1/a/duty_cycle', 'w') as filetowrite:
        filetowrite.write(str(int(dont_move * FACTOR)))

def go():
    """
    Sends the car forward at a default PWM
    :return: none
    """
    with open('/dev/bone/pwm/1/a/duty_cycle', 'w') as filetowrite:
        filetowrite.write(str(int(go_forward * FACTOR)))

def go_faster():
    """
    Sends the car forward at a faster default PWM
    :return: none
    """
    with open('/dev/bone/pwm/1/a/duty_cycle', 'w') as filetowrite:
        filetowrite.write(str(int((go_forward + go_faster_addition) * FACTOR)))

def detect_edges(frame):
    # filter for blue lane lines
    hsv = cv2.cvtColor(frame, cv2.COLOR_BGR2HSV)
    # cv2.imshow("HSV",hsv)
    lower_blue = np.array([90, 120, 0], dtype="uint8")
    upper_blue = np.array([150, 255, 255], dtype="uint8")
    mask = cv2.inRange(hsv, lower_blue, upper_blue)
    # cv2.imshow("mask",mask)

    # detect edges
    edges = cv2.Canny(mask, 50, 100)
    # cv2.imshow("edges",edges)

    return edges

def region_of_interest(edges):
    height, width = edges.shape
    mask = np.zeros_like(edges)

    # only focus lower half of the screen
    polygon = np.array([[
        (0, height),
        (0, height / 2),
        (width, height / 2),
        (width, height),
    ]], np.int32)

    cv2.fillPoly(mask, polygon, 255)

    cropped_edges = cv2.bitwise_and(edges, mask)
    # cv2.imshow("roi",cropped_edges)

    return cropped_edges

def detect_line_segments(cropped_edges):
    rho = 1
    theta = np.pi / 180
    min_threshold = 10

    line_segments = cv2.HoughLinesP(cropped_edges, rho, theta, min_threshold,
                                    np.array([]), minLineLength=5, maxLineGap=150)

    return line_segments

def average_slope_intercept(frame, line_segments):
    lane_lines = []

    if line_segments is None:
        print("no line segments detected")
        return lane_lines

    height, width, _ = frame.shape
    left_fit = []
    right_fit = []

    boundary = 1 / 3
    left_region_boundary = width * (1 - boundary)
    right_region_boundary = width * boundary

    for line_segment in line_segments:
        for x1, y1, x2, y2 in line_segment:
            if x1 == x2:
                print("skipping vertical lines (slope = infinity")
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

def display_lines(frame, lines, line_color=(0, 255, 0), line_width=6):
    line_image = np.zeros_like(frame)

    if lines is not None:
        for line in lines:
            for x1, y1, x2, y2 in line:
                cv2.line(line_image, (x1, y1), (x2, y2), line_color, line_width)

    line_image = cv2.addWeighted(frame, 0.8, line_image, 1, 1)

    return line_image

def display_heading_line(frame, steering_angle, line_color=(0, 0, 255), line_width=5):
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

def get_steering_angle(frame, lane_lines):
    height, width, _ = frame.shape

    if len(lane_lines) == 2:
        _, _, left_x2, _ = lane_lines[0][0]
        _, _, right_x2, _ = lane_lines[1][0]
        mid = int(width / 2)
        x_offset = (left_x2 + right_x2) / 2 - mid
        y_offset = int(height / 2)

    elif len(lane_lines) == 1:
        x1, _, x2, _ = lane_lines[0][0]
        x_offset = x2 - x1
        y_offset = int(height / 2)

    elif len(lane_lines) == 0:
        x_offset = 0
        y_offset = int(height / 2)

    angle_to_mid_radian = math.atan(x_offset / y_offset)
    angle_to_mid_deg = int(angle_to_mid_radian * 180.0 / math.pi)
    steering_angle = angle_to_mid_deg + 90
    #print("Steering angle " + str(steering_angle))
    return steering_angle

# set up the car throttle and steering PWMs
initialize_car()

# set up video
video = cv2.VideoCapture(2)
video.set(cv2.CAP_PROP_FRAME_WIDTH, 320)
video.set(cv2.CAP_PROP_FRAME_HEIGHT, 240)

# wait for video to load
time.sleep(1)

# PD variables
kp = 0.085
kd = kp * 0.15
kd = 0
lastTime = 0
lastError = 0

# counter for number of ticks
counter = 0

# start the engines
go()

# arrays for making the final graphs
speed_pwm = []
steer_pwm = []
current_speed = go_forward

sightDebug = True

stopSignCheck = 1
isStopSignBool = False
passedStopLight = False
stopSignTick = -1

# Max number of loops
max_ticks = 2000

while counter < max_ticks:

    ret, original_frame = video.read()

    frame = cv2.resize(original_frame, (160, 120))

    if sightDebug:
        cv2.imshow("Resized Frame", frame)

    # check for stop sign/traffic light every couple ticks
    if ((counter + 1) % stopSignCheck) == 0:
        # if the counter has done 200 ticks, then check again for stop sign
        if counter > stopSignTick:
	    
            isStopSignBool, floorSight = isRedFloorVisible(frame)

            if sightDebug:
                cv2.imshow("floorSight", floorSight)

            # If a stop sign is detected, then stop it and sleep for 2 seconds
            if isStopSignBool:
                print("Detected stop sign, stopping now.")
                time.sleep(0.1)
                stop()
                time.sleep(2)
                stopSignTick = counter + 200
                print("Stop sign finished.")
                passedStopLight = True

    # If the car has just passed a stop light, then it goes again
    if passedStopLight:
        go()
        time.sleep(1)
        passedStopLight = False

    # process the frame to determine the desired steering angle
    # cv2.imshow("original",frame)
    edges = detect_edges(frame)
    roi = region_of_interest(edges)
    line_segments = detect_line_segments(roi)
    lane_lines = average_slope_intercept(frame, line_segments)
    lane_lines_image = display_lines(frame, lane_lines)
    steering_angle = get_steering_angle(frame, lane_lines)
    # heading_image = display_heading_line(lane_lines_image,steering_angle)
    # cv2.imshow("heading line",heading_image)

    # calculate changes for PD
    now = time.time()
    dt = now - lastTime
    if sightDebug:
        cv2.imshow("Cropped sight", roi)
    deviation = steering_angle - 90

    # PD Code
    error = -deviation
    base_turn = 7.5
    proportional = kp * error
    derivative = kd * (error - lastError) / dt

    # determine actual turn to do
    turn_amt = base_turn + proportional + derivative
    # caps turns to make PWM values
    if 7.0 < turn_amt < 7.8:
        turn_amt = 7.5
    elif turn_amt > left:
        print("Turning Left")
        turn_amt = left
    elif turn_amt < right:
        print("Turning right")
        turn_amt = right

    # turn!
    with open('/dev/bone/pwm/1/b/duty_cycle', 'w') as filetowrite:
        filetowrite.write(str(int(turn_amt * FACTOR)))
   
    # take values for graphs
    steer_pwm.append(turn_amt)
    speed_pwm.append(current_speed)

    # update PD values for next loop
    lastError = error
    lastTime = time.time()

    key = cv2.waitKey(1)
    if key == 27:
        break

    counter += 1

# clean up resources
video.release()
cv2.destroyAllWindows()
with open('/dev/bone/pwm/1/a/duty_cycle', 'w') as filetowrite:
    filetowrite.write('1500000')
with open('/dev/bone/pwm/1/b/duty_cycle', 'w') as filetowrite:
    filetowrite.write('1500000')

"""
