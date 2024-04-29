#!/usr/bin/env python
# -*- coding: utf-8 -*-
import csv
import copy
import argparse
import itertools
from collections import Counter
from collections import deque
import picamera2 
import time
import cv2 as cv
import numpy as np
import mediapipe as mp
import httpx
import time
from utils import CvFpsCalc
from model import KeyPointClassifier
from model import PointHistoryClassifier

# for retreat
# IP_ADDR = '10.2.78.10'

# for illinois net
# IP_ADDR = '172.16.100.169'

# for PCB v1
IP_ADDR = '172.16.103.137'

# for PCB v3

url = f"http://{IP_ADDR}"

# for /move
move1gridData = {
    'duration': 3000,
    'speed': 35,
    'direction': 'forward'
}

movehalfgridData = {
    'duration': 2500,
    'speed': 35,
    'direction': 'forward'
}
move2gridData = {
    'duration': 4000,
    'speed': 35,
    'direction': 'forward'
}

turnleftData = {
     'duration': 1500,
     'speed': 20,
     'direction': 'left'
}

# for /turn, 90 degree right
turnrightData = {
    'duration': 1500,
    'speed': 20,
    'direction': 'right'
}
 #for /turn, 180 degree
turnData_180 = {
    'duration': 3200,
    'speed': 20,
    'direction': 'left'
}
def syncPOST(rel_Path, data): 
    try:
        response = httpx.post(url+rel_Path, data=data)
        if response.status_code == 200:
            (f"send to {url+rel_Path}")
            # print(response.text)
        else:
            print(f"failed to send to {IP_ADDR}, status: {response.status_code}")
            return 
        
    except httpx.RequestError as e:
        # print(url+rel_Path)
        
        # print(f"Request error: {e}")
        return




def get_args():
    parser = argparse.ArgumentParser()

    parser.add_argument("--device", type=int, default=0)
    parser.add_argument("--width", help='cap width', type=int, default=960)
    parser.add_argument("--height", help='cap height', type=int, default=540)

    parser.add_argument('--use_static_image_mode', action='store_true')
    parser.add_argument("--min_detection_confidence",
                        help='min_detection_confidence',
                        type=float,
                        default=0.7)
    parser.add_argument("--min_tracking_confidence",
                        help='min_tracking_confidence',
                        type=int,
                        default=0.5)

    args = parser.parse_args()

    return args


def main():
    # Argument parsing #################################################################
    args = get_args()

    cap_device = args.device
    cap_width = args.width
    cap_height = args.height

    use_static_image_mode = args.use_static_image_mode
    min_detection_confidence = args.min_detection_confidence
    min_tracking_confidence = args.min_tracking_confidence

    use_brect = True
    counter = 0
    # Camera preparation ###############################################################
    ''' 
    cap = cv.VideoCapture(cap_device)
    cap.set(cv.CAP_PROP_FRAME_WIDTH, cap_width)
    cap.set(cv.CAP_PROP_FRAME_HEIGHT, cap_height)
    '''
    
    cv.startWindowThread()
    camera = picamera2.Picamera2()
    camera.configure(camera.create_preview_configuration(main={"format": 'XRGB8888', "size":(880, 660)}))
    camera.start()
    # Model load #############################################################
    mp_hands = mp.solutions.hands
    hands = mp_hands.Hands(
        static_image_mode=use_static_image_mode,
        max_num_hands=1,
        min_detection_confidence=min_detection_confidence,
        min_tracking_confidence=min_tracking_confidence,
    )

    keypoint_classifier = KeyPointClassifier()

    point_history_classifier = PointHistoryClassifier()

    # Read labels ###########################################################
    with open('model/keypoint_classifier/keypoint_classifier_label.csv',
              encoding='utf-8-sig') as f:
        keypoint_classifier_labels = csv.reader(f)
        keypoint_classifier_labels = [
            row[0] for row in keypoint_classifier_labels
        ]
    with open(
            'model/point_history_classifier/point_history_classifier_label.csv',
            encoding='utf-8-sig') as f:
        point_history_classifier_labels = csv.reader(f)
        point_history_classifier_labels = [
            row[0] for row in point_history_classifier_labels
        ]

    # FPS Measurement ########################################################
    cvFpsCalc = CvFpsCalc(buffer_len=10)

    # Coordinate history #################################################################
    history_length = 16
    point_history = deque(maxlen=history_length)

    # Finger gesture history ################################################
    finger_gesture_history = deque(maxlen=history_length)

    #  ########################################################################
    
    mode = 0
    #rrom_size is the size of a square in cm
    room_size = 490
    #discretize the room into square
    check = np.zeros((6,9))
    rest_row = 1
    rest_col = 4
    trash_curr_row = 1
    trash_curr_col = 4
    while True:
        fps = cvFpsCalc.get()

        # Process Key (ESC: end) #################################################
        key = cv.waitKey(10)
        if key == 27:  # ESC
            break
        number, mode = select_mode(key, mode)

        # Camera capture #####################################################
        image = camera.capture_array()
        if len(image) == 0:
            #print("failed to grab frame")
            break
        image = cv.flip(image, 1)  # Mirror display
        debug_image = copy.deepcopy(image)
        # print(debug_image.shape)

        # Detection implementation #############################################################
        image = cv.cvtColor(image, cv.COLOR_BGR2RGB)

        image.flags.writeable = False
        results = hands.process(image)
        image.flags.writeable = True

        #  ####################################################################
        if results.multi_hand_landmarks is not None:
            for hand_landmarks, handedness in zip(results.multi_hand_landmarks,
                                                  results.multi_handedness):
                # Bounding box calculation
                brect = calc_bounding_rect(debug_image, hand_landmarks)
                # brect is the position
                x = (brect[0] + brect[2]) / 2
                y = (brect[1] + brect[3]) / 2
                row = int(y/110)  # get current row
                col = int(x/110)

                # Landmark calculation
                landmark_list = calc_landmark_list(debug_image, hand_landmarks)
                
                # Conversion to relative coordinates / normalized coordinates
                pre_processed_landmark_list = pre_process_landmark(
                    landmark_list)
                pre_processed_point_history_list = pre_process_point_history(
                    debug_image, point_history)
                # Write to the dataset file
                logging_csv(number, mode, pre_processed_landmark_list,
                            pre_processed_point_history_list)

                # Hand sign classification
                hand_sign_id = keypoint_classifier(pre_processed_landmark_list)

                #print(hand_sign_id)
                '''if hand_sign_id != 5:
                    counter = 0
                if hand_sign_id == 5:
                    counter = counter + 1
                    if counter < 7:
                        continue
                    #send the command to bin once the sign is stable
                    else:
                        #print("send signal")
                        counter = 0


                        #determine color
                        color = determine_color(row,col)
                        #print(row,col)
                        if color == 'obstacle':
                            color = 'yellow'
                        #this line is hard code testing
                        #color = 'blue'; row = 1; col = 4
                        # #print(color)
                        
                        # #lead bin to place
                        #print("send signal to the bin",color)
                        
                        lead_bin_to_destination(row,col,1,4,color)
                        
                        syncPOST('/set_lid', None)
                        time.sleep(60)
                        # #take the bin back
                        lead_bin_to_rest(trash_curr_row,trash_curr_col,row,col,color)'''
                        


                if hand_sign_id == 2:  # Point gesture
                    point_history.append(landmark_list[8])
                else:
                    point_history.append([0, 0])
                
                # Finger gesture classification
                finger_gesture_id = 0
                point_history_len = len(pre_processed_point_history_list)
                if point_history_len == (history_length * 2):
                    finger_gesture_id = point_history_classifier(
                        pre_processed_point_history_list)

                # Calculates the gesture IDs in the latest detection
                finger_gesture_history.append(finger_gesture_id)
                most_common_fg_id = Counter(
                    finger_gesture_history).most_common()

                # Drawing part
                
                debug_image = draw_bounding_rect(use_brect, debug_image, brect)
                debug_image = draw_landmarks(debug_image, landmark_list)
                debug_image = draw_info_text(
                    debug_image,
                    brect,
                    handedness,
                    keypoint_classifier_labels[hand_sign_id],
                    point_history_classifier_labels[most_common_fg_id[0][0]],
                )
        else:
            point_history.append([0, 0])

        debug_image = draw_point_history(debug_image, point_history)
        debug_image = draw_info(debug_image, fps, mode, number)

        # Screen reflection #############################################################
        cv.imshow('Hand Gesture Recognition', debug_image)

    #cap.release()
    cv.destroyAllWindows()

#determine the color of the belonging place
def determine_color(row,col):
    x = row
    y = col
    if x >= 0 and x <= 1 and y >= 0 and y <= 5:
        return 'green'
    if x >= 2 and x <= 5 and y >= 0 and y <= 3:
        return 'blue'
    if x >= 4 and x <= 5 and y >= 4 and y <= 5:
        return 'yellow'
    if x >= 8 and x <= 8 and y >= 4 and y <= 5:
        return 'red'
    if x >= 7 and x <= 8 and y >= 0 and y <= 3:
        return 'pink'
    return 'obstacle'


def lead_bin_to_destination(row,col,trash_curr_row,trash_curr_col,color):
    if color == 'green':
        #go along y-axis, for ( 4-row ) * 35cm
        for i in range(1):
            syncPOST('/move', move1gridData)
            time.sleep(10)
    if color == 'blue':
        #go to 1 and go along x axis
        go_start_to_1()
        #for ( col - 1) * 35cm
        for i in range(1):
            syncPOST('/move', move1gridData)
            time.sleep(10)
        '''if 1 - row > 0:
            #turn left
            syncPOST('/turn', turnleftData)
            time.sleep(3)
        if 1 - row < 0:
            #turn right
            syncPOST('/turn', turnrightData)
            time.sleep(3)
        #for (1 - row) * 35cm
        if row - 1 != 0:
            #syncPOST('/move', move1gridData)
            time.sleep(2.2)'''
        

    if color == 'yellow':
        #go to 1,2,3
        go_start_to_1()
        go_1_to_2()
        go_2_to_3()
        
    if color == 'red':
        #go to 1,2,3 and then move along x axis
        #for (col - 4) * 35cm
        go_start_to_1()
        go_1_to_2()
        go_2_to_3()
        syncPOST('/turn', turnleftData)
        time.sleep(5)
        for i in range(1):
            syncPOST('/move', move1gridData)
            time.sleep(10)
        
    if color == 'pink':
        go_start_to_1()
        go_1_to_2()
        go_2_to_3()
        syncPOST('/turn', turnleftData)
        time.sleep(5)
        go_3_to_4()
        syncPOST('/turn', turnleftData)
        time.sleep(5)
        #go to 1,2,3,4 and then move along y axis
        #for(4 - row) * 35 cm
        for i in range(1):
            syncPOST('/move', move1gridData)
            time.sleep(10)
        
    return

#heading leading comman. 
 
def go_start_to_1():
    for i in range(1):
        syncPOST('/move', move1gridData)
        time.sleep(10)
    #move 3*35cm ahead
    #turn right
    syncPOST('/turn', turnrightData)
    time.sleep(5)
def go_1_to_2():
    #move 35*4cm forwad, turn right
    for i in range(1):
        syncPOST('/move', move2gridData)
        time.sleep(10)
    
    syncPOST('/turn', turnrightData)
    time.sleep(5)
def go_2_to_3():
    #move 35*3cm forwad
    for i in range(1):
        syncPOST('/move', movehalfgridData)
        time.sleep(10)
    return
def go_3_to_4():
    # move 35*3cm forwad,turn left
    for i in range(1):
        syncPOST('/move', move1gridData)
        time.sleep(10)
    return


#return leading command
def go_1_to_start():
    #move 3*35cm ahead
    for i in range(1):
        syncPOST('/move', move1gridData)
        time.sleep(10)
    return
def go_2_to_1():
    #move 35*4cm forwad, turn left
    for i in range(1):
        syncPOST('/move', move2gridData)
        time.sleep(10)
    syncPOST('/turn', turnleftData)
    time.sleep(5)
    return
def go_3_to_2():
    # move 35*3cm forwad, turn left
    for i in range(1):
        syncPOST('/move', movehalfgridData)
        time.sleep(10)
    syncPOST('/turn', turnleftData)
    time.sleep(5)
    return
def go_4_to_3():
    #move 35*3cm forwad, turn right
    for i in range(1):
        syncPOST('/move', move1gridData)
        time.sleep(10)
    syncPOST('/turn', turnrightData)
    time.sleep(5)
    return

def turnaround():
    
    syncPOST('/turn', turnData_180)
    time.sleep(5)
def lead_bin_to_rest(trash_curr_row,trash_curr_col,row,col,color):
    if color == 'green':
        #turn around
        turnaround()
        #move (4-row ) * 35cm
        for i in range(1):
            syncPOST('/move', move1gridData)
            time.sleep(10)
    
    if color == 'blue':
        turnaround()
        #, move (col -1)*35 cm
        '''if col > 1:
            syncPOST('/turn', turnrightData)
            time.sleep(3)
        if col < 1:
            syncPOST('/turn', turnleftData)
            time.sleep(3)  
        if col == 1:
            turnaround()'''
        for i in range(1):
            syncPOST('/move', move1gridData)
            time.sleep(10)
        #turn left
        syncPOST('/turn', turnleftData)
        time.sleep(5)
        #move (4-row ) * 35cm
        for i in range(1):
            syncPOST('/move', move1gridData)
            time.sleep(10)
        
        
    if color == 'yellow':
        #turn around
        turnaround()
        go_3_to_2()
        go_2_to_1()
        go_1_to_start()
        #go to 2,1 and then move along y axis
        
    if color == 'red':
        #turn around
        turnaround()
        #for (col - 4) * 35cm
        for i in range(1):
            syncPOST('/move', move1gridData)
            time.sleep(10)
        syncPOST('/turn', turnrightData)
        time.sleep(5)
        #go to 2,1 and then move along y axis
        go_3_to_2()
        go_2_to_1()
        go_1_to_start()
        
    if color == 'pink':
        #turn around, #for(4 - row) * 35 cm,turn right
        turnaround()
        for i in range(1):
            syncPOST('/move', move1gridData)
            time.sleep(10)
        syncPOST('/turn', turnrightData)
        time.sleep(5)
        #go to 3,2,1 and then move along y axis
        go_4_to_3()
        go_3_to_2()
        go_2_to_1()
        go_1_to_start()
        
    #let the bin turn arund
    if color != 'obstacle':
        turnaround()
    return

def select_mode(key, mode):
    number = -1
    if 48 <= key <= 57:  # 0 ~ 9
        number = key - 48
    if key == 110:  # n
        mode = 0
    if key == 107:  # k
        mode = 1
    if key == 104:  # h
        mode = 2
    return number, mode


def calc_bounding_rect(image, landmarks):
    image_width, image_height = image.shape[1], image.shape[0]

    landmark_array = np.empty((0, 2), int)

    for _, landmark in enumerate(landmarks.landmark):
        landmark_x = min(int(landmark.x * image_width), image_width - 1)
        landmark_y = min(int(landmark.y * image_height), image_height - 1)

        landmark_point = [np.array((landmark_x, landmark_y))]

        landmark_array = np.append(landmark_array, landmark_point, axis=0)

    x, y, w, h = cv.boundingRect(landmark_array)

    return [x, y, x + w, y + h]


def calc_landmark_list(image, landmarks):
    image_width, image_height = image.shape[1], image.shape[0]

    landmark_point = []

    # Keypoint
    for _, landmark in enumerate(landmarks.landmark):
        landmark_x = min(int(landmark.x * image_width), image_width - 1)
        landmark_y = min(int(landmark.y * image_height), image_height - 1)
        # landmark_z = landmark.z

        landmark_point.append([landmark_x, landmark_y])

    return landmark_point


def pre_process_landmark(landmark_list):
    temp_landmark_list = copy.deepcopy(landmark_list)

    # Convert to relative coordinates
    base_x, base_y = 0, 0
    for index, landmark_point in enumerate(temp_landmark_list):
        if index == 0:
            base_x, base_y = landmark_point[0], landmark_point[1]

        temp_landmark_list[index][0] = temp_landmark_list[index][0] - base_x
        temp_landmark_list[index][1] = temp_landmark_list[index][1] - base_y

    # Convert to a one-dimensional list
    temp_landmark_list = list(
        itertools.chain.from_iterable(temp_landmark_list))

    # Normalization
    max_value = max(list(map(abs, temp_landmark_list)))

    def normalize_(n):
        return n / max_value

    temp_landmark_list = list(map(normalize_, temp_landmark_list))

    return temp_landmark_list


def pre_process_point_history(image, point_history):
    image_width, image_height = image.shape[1], image.shape[0]

    temp_point_history = copy.deepcopy(point_history)

    # Convert to relative coordinates
    base_x, base_y = 0, 0
    for index, point in enumerate(temp_point_history):
        if index == 0:
            base_x, base_y = point[0], point[1]

        temp_point_history[index][0] = (temp_point_history[index][0] -
                                        base_x) / image_width
        temp_point_history[index][1] = (temp_point_history[index][1] -
                                        base_y) / image_height

    # Convert to a one-dimensional list
    temp_point_history = list(
        itertools.chain.from_iterable(temp_point_history))

    return temp_point_history


def logging_csv(number, mode, landmark_list, point_history_list):
    if mode == 0:
        pass
    if mode == 1 and (0 <= number <= 9):
        csv_path = 'model/keypoint_classifier/keypoint.csv'
        with open(csv_path, 'a', newline="") as f:
            writer = csv.writer(f)
            writer.writerow([number, *landmark_list])
    if mode == 2 and (0 <= number <= 9):
        csv_path = 'model/point_history_classifier/point_history.csv'
        with open(csv_path, 'a', newline="") as f:
            writer = csv.writer(f)
            writer.writerow([number, *point_history_list])
    return


def draw_landmarks(image, landmark_point):
    if len(landmark_point) > 0:
        # Thumb
        cv.line(image, tuple(landmark_point[2]), tuple(landmark_point[3]),
                (0, 0, 0), 6)
        cv.line(image, tuple(landmark_point[2]), tuple(landmark_point[3]),
                (255, 255, 255), 2)
        cv.line(image, tuple(landmark_point[3]), tuple(landmark_point[4]),
                (0, 0, 0), 6)
        cv.line(image, tuple(landmark_point[3]), tuple(landmark_point[4]),
                (255, 255, 255), 2)

        # Index finger
        cv.line(image, tuple(landmark_point[5]), tuple(landmark_point[6]),
                (0, 0, 0), 6)
        cv.line(image, tuple(landmark_point[5]), tuple(landmark_point[6]),
                (255, 255, 255), 2)
        cv.line(image, tuple(landmark_point[6]), tuple(landmark_point[7]),
                (0, 0, 0), 6)
        cv.line(image, tuple(landmark_point[6]), tuple(landmark_point[7]),
                (255, 255, 255), 2)
        cv.line(image, tuple(landmark_point[7]), tuple(landmark_point[8]),
                (0, 0, 0), 6)
        cv.line(image, tuple(landmark_point[7]), tuple(landmark_point[8]),
                (255, 255, 255), 2)

        # Middle finger
        cv.line(image, tuple(landmark_point[9]), tuple(landmark_point[10]),
                (0, 0, 0), 6)
        cv.line(image, tuple(landmark_point[9]), tuple(landmark_point[10]),
                (255, 255, 255), 2)
        cv.line(image, tuple(landmark_point[10]), tuple(landmark_point[11]),
                (0, 0, 0), 6)
        cv.line(image, tuple(landmark_point[10]), tuple(landmark_point[11]),
                (255, 255, 255), 2)
        cv.line(image, tuple(landmark_point[11]), tuple(landmark_point[12]),
                (0, 0, 0), 6)
        cv.line(image, tuple(landmark_point[11]), tuple(landmark_point[12]),
                (255, 255, 255), 2)

        # Ring finger
        cv.line(image, tuple(landmark_point[13]), tuple(landmark_point[14]),
                (0, 0, 0), 6)
        cv.line(image, tuple(landmark_point[13]), tuple(landmark_point[14]),
                (255, 255, 255), 2)
        cv.line(image, tuple(landmark_point[14]), tuple(landmark_point[15]),
                (0, 0, 0), 6)
        cv.line(image, tuple(landmark_point[14]), tuple(landmark_point[15]),
                (255, 255, 255), 2)
        cv.line(image, tuple(landmark_point[15]), tuple(landmark_point[16]),
                (0, 0, 0), 6)
        cv.line(image, tuple(landmark_point[15]), tuple(landmark_point[16]),
                (255, 255, 255), 2)

        # Little finger
        cv.line(image, tuple(landmark_point[17]), tuple(landmark_point[18]),
                (0, 0, 0), 6)
        cv.line(image, tuple(landmark_point[17]), tuple(landmark_point[18]),
                (255, 255, 255), 2)
        cv.line(image, tuple(landmark_point[18]), tuple(landmark_point[19]),
                (0, 0, 0), 6)
        cv.line(image, tuple(landmark_point[18]), tuple(landmark_point[19]),
                (255, 255, 255), 2)
        cv.line(image, tuple(landmark_point[19]), tuple(landmark_point[20]),
                (0, 0, 0), 6)
        cv.line(image, tuple(landmark_point[19]), tuple(landmark_point[20]),
                (255, 255, 255), 2)

        # Palm
        cv.line(image, tuple(landmark_point[0]), tuple(landmark_point[1]),
                (0, 0, 0), 6)
        cv.line(image, tuple(landmark_point[0]), tuple(landmark_point[1]),
                (255, 255, 255), 2)
        cv.line(image, tuple(landmark_point[1]), tuple(landmark_point[2]),
                (0, 0, 0), 6)
        cv.line(image, tuple(landmark_point[1]), tuple(landmark_point[2]),
                (255, 255, 255), 2)
        cv.line(image, tuple(landmark_point[2]), tuple(landmark_point[5]),
                (0, 0, 0), 6)
        cv.line(image, tuple(landmark_point[2]), tuple(landmark_point[5]),
                (255, 255, 255), 2)
        cv.line(image, tuple(landmark_point[5]), tuple(landmark_point[9]),
                (0, 0, 0), 6)
        cv.line(image, tuple(landmark_point[5]), tuple(landmark_point[9]),
                (255, 255, 255), 2)
        cv.line(image, tuple(landmark_point[9]), tuple(landmark_point[13]),
                (0, 0, 0), 6)
        cv.line(image, tuple(landmark_point[9]), tuple(landmark_point[13]),
                (255, 255, 255), 2)
        cv.line(image, tuple(landmark_point[13]), tuple(landmark_point[17]),
                (0, 0, 0), 6)
        cv.line(image, tuple(landmark_point[13]), tuple(landmark_point[17]),
                (255, 255, 255), 2)
        cv.line(image, tuple(landmark_point[17]), tuple(landmark_point[0]),
                (0, 0, 0), 6)
        cv.line(image, tuple(landmark_point[17]), tuple(landmark_point[0]),
                (255, 255, 255), 2)

    # Key Points
    for index, landmark in enumerate(landmark_point):
        if index == 0:  # 手首1
            cv.circle(image, (landmark[0], landmark[1]), 5, (255, 255, 255),
                      -1)
            cv.circle(image, (landmark[0], landmark[1]), 5, (0, 0, 0), 1)
        if index == 1:  # 手首2
            cv.circle(image, (landmark[0], landmark[1]), 5, (255, 255, 255),
                      -1)
            cv.circle(image, (landmark[0], landmark[1]), 5, (0, 0, 0), 1)
        if index == 2:  # 親指：付け根
            cv.circle(image, (landmark[0], landmark[1]), 5, (255, 255, 255),
                      -1)
            cv.circle(image, (landmark[0], landmark[1]), 5, (0, 0, 0), 1)
        if index == 3:  # 親指：第1関節
            cv.circle(image, (landmark[0], landmark[1]), 5, (255, 255, 255),
                      -1)
            cv.circle(image, (landmark[0], landmark[1]), 5, (0, 0, 0), 1)
        if index == 4:  # 親指：指先
            cv.circle(image, (landmark[0], landmark[1]), 8, (255, 255, 255),
                      -1)
            cv.circle(image, (landmark[0], landmark[1]), 8, (0, 0, 0), 1)
        if index == 5:  # 人差指：付け根
            cv.circle(image, (landmark[0], landmark[1]), 5, (255, 255, 255),
                      -1)
            cv.circle(image, (landmark[0], landmark[1]), 5, (0, 0, 0), 1)
        if index == 6:  # 人差指：第2関節
            cv.circle(image, (landmark[0], landmark[1]), 5, (255, 255, 255),
                      -1)
            cv.circle(image, (landmark[0], landmark[1]), 5, (0, 0, 0), 1)
        if index == 7:  # 人差指：第1関節
            cv.circle(image, (landmark[0], landmark[1]), 5, (255, 255, 255),
                      -1)
            cv.circle(image, (landmark[0], landmark[1]), 5, (0, 0, 0), 1)
        if index == 8:  # 人差指：指先
            cv.circle(image, (landmark[0], landmark[1]), 8, (255, 255, 255),
                      -1)
            cv.circle(image, (landmark[0], landmark[1]), 8, (0, 0, 0), 1)
        if index == 9:  # 中指：付け根
            cv.circle(image, (landmark[0], landmark[1]), 5, (255, 255, 255),
                      -1)
            cv.circle(image, (landmark[0], landmark[1]), 5, (0, 0, 0), 1)
        if index == 10:  # 中指：第2関節
            cv.circle(image, (landmark[0], landmark[1]), 5, (255, 255, 255),
                      -1)
            cv.circle(image, (landmark[0], landmark[1]), 5, (0, 0, 0), 1)
        if index == 11:  # 中指：第1関節
            cv.circle(image, (landmark[0], landmark[1]), 5, (255, 255, 255),
                      -1)
            cv.circle(image, (landmark[0], landmark[1]), 5, (0, 0, 0), 1)
        if index == 12:  # 中指：指先
            cv.circle(image, (landmark[0], landmark[1]), 8, (255, 255, 255),
                      -1)
            cv.circle(image, (landmark[0], landmark[1]), 8, (0, 0, 0), 1)
        if index == 13:  # 薬指：付け根
            cv.circle(image, (landmark[0], landmark[1]), 5, (255, 255, 255),
                      -1)
            cv.circle(image, (landmark[0], landmark[1]), 5, (0, 0, 0), 1)
        if index == 14:  # 薬指：第2関節
            cv.circle(image, (landmark[0], landmark[1]), 5, (255, 255, 255),
                      -1)
            cv.circle(image, (landmark[0], landmark[1]), 5, (0, 0, 0), 1)
        if index == 15:  # 薬指：第1関節
            cv.circle(image, (landmark[0], landmark[1]), 5, (255, 255, 255),
                      -1)
            cv.circle(image, (landmark[0], landmark[1]), 5, (0, 0, 0), 1)
        if index == 16:  # 薬指：指先
            cv.circle(image, (landmark[0], landmark[1]), 8, (255, 255, 255),
                      -1)
            cv.circle(image, (landmark[0], landmark[1]), 8, (0, 0, 0), 1)
        if index == 17:  # 小指：付け根
            cv.circle(image, (landmark[0], landmark[1]), 5, (255, 255, 255),
                      -1)
            cv.circle(image, (landmark[0], landmark[1]), 5, (0, 0, 0), 1)
        if index == 18:  # 小指：第2関節
            cv.circle(image, (landmark[0], landmark[1]), 5, (255, 255, 255),
                      -1)
            cv.circle(image, (landmark[0], landmark[1]), 5, (0, 0, 0), 1)
        if index == 19:  # 小指：第1関節
            cv.circle(image, (landmark[0], landmark[1]), 5, (255, 255, 255),
                      -1)
            cv.circle(image, (landmark[0], landmark[1]), 5, (0, 0, 0), 1)
        if index == 20:  # 小指：指先
            cv.circle(image, (landmark[0], landmark[1]), 8, (255, 255, 255),
                      -1)
            cv.circle(image, (landmark[0], landmark[1]), 8, (0, 0, 0), 1)

    return image


def draw_bounding_rect(use_brect, image, brect):
    if use_brect:
        # Outer rectangle
        cv.rectangle(image, (brect[0], brect[1]), (brect[2], brect[3]),
                     (0, 0, 0), 1)
    # print(brect)
    return image


def draw_info_text(image, brect, handedness, hand_sign_text,
                   finger_gesture_text):
    cv.rectangle(image, (brect[0], brect[1]), (brect[2], brect[1] - 22),
                 (0, 0, 0), -1)

    info_text = handedness.classification[0].label[0:]
    if hand_sign_text != "":
        info_text = info_text + ':' + hand_sign_text
    cv.putText(image, info_text, (brect[0] + 5, brect[1] - 4),
               cv.FONT_HERSHEY_SIMPLEX, 0.6, (255, 255, 255), 1, cv.LINE_AA)

    if finger_gesture_text != "":
        cv.putText(image, "Finger Gesture:" + finger_gesture_text, (10, 60),
                   cv.FONT_HERSHEY_SIMPLEX, 1.0, (0, 0, 0), 4, cv.LINE_AA)
        cv.putText(image, "Finger Gesture:" + finger_gesture_text, (10, 60),
                   cv.FONT_HERSHEY_SIMPLEX, 1.0, (255, 255, 255), 2,
                   cv.LINE_AA)

    return image


def draw_point_history(image, point_history):
    for index, point in enumerate(point_history):
        if point[0] != 0 and point[1] != 0:
            cv.circle(image, (point[0], point[1]), 1 + int(index / 2),
                      (152, 251, 152), 2)

    return image


def draw_info(image, fps, mode, number):
    '''
    cv.putText(image, "FPS:" + str(fps), (10, 30), cv.FONT_HERSHEY_SIMPLEX,
               1.0, (0, 0, 0), 4, cv.LINE_AA)
    cv.putText(image, "FPS:" + str(fps), (10, 30), cv.FONT_HERSHEY_SIMPLEX,
               1.0, (255, 255, 255), 2, cv.LINE_AA)
    '''

    mode_string = ['Logging Key Point', 'Logging Point History']
    if 1 <= mode <= 2:
        cv.putText(image, "MODE:" + mode_string[mode - 1], (10, 90),
                   cv.FONT_HERSHEY_SIMPLEX, 0.6, (255, 255, 255), 1,
                   cv.LINE_AA)
        if 0 <= number <= 9:
            cv.putText(image, "NUM:" + str(number), (10, 110),
                       cv.FONT_HERSHEY_SIMPLEX, 0.6, (255, 255, 255), 1,
                       cv.LINE_AA)
    return image


if __name__ == '__main__':
    main()
