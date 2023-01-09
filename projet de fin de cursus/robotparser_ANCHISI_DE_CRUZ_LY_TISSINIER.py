# robotparser
# Program intended to run on robots equipped with raspberry pi, frontal camera with motor to rotate, motors on both back wheels
# and an Arduina card to control them. Will only work with a computer acting as a server, with another robot equipped with this
# program and the corresponding Arduino program flashed on the Arduino.
# Authors: Ambre ANCHISI, Ocean DE CRUZ, Alexandre LY, Axel TISSINIER
# IPSA SAA AU516 Final project
# 5 Jan 2023

# Imports
import cv2                      # Image processing
from cv2 import aruco           # Aruco detection
from picamera import PiCamera   # Camera
import serial                   # Communication with arduino
import numpy as np              # Matrix operations for images
import argparse                 # Parser
import socket                   # Communication between robots
import threading                # Multiple thread
import time                     # Pauses in program


# Argument parser
def parse_opt():
    # construct the argument parser
    parser = argparse.ArgumentParser(description='Script to run the ball detection.')
    
    # Arguments
    parser.add_argument("--serverIP", default="NOSERVER", type=str, help="Server IP.")
    parser.add_argument("--serverPort", default=5000, type=int, help="Server port for communication.")
    parser.add_argument("--lowThresh", default=(100,100,50), help="Lower argument of HSV threshold.")
    parser.add_argument("--highThresh", default=(255,255,255), help='Higher argument of HSV threshold.')
    parser.add_argument("--minDist", default=30, type=int, help='Minimum distance between circles centers.')
    parser.add_argument("--param1", default=60, type=int, help='First parameter of Hough circles: correspond to canny filter threshold.')
    parser.add_argument("--param2", default=25, type=int, help='Second parameter of Hough circles: correspond to how much shapes must be circle like to be detected.')
    parser.add_argument("--minRad", default=40, type=int, help='Minimum radius of detected circles.')
    parser.add_argument("--maxRad", default=200, type=int, help='Maximum radius of detected circles.')
    parser.add_argument("--blurRad", default=5, type=int, help='Radius of Gaussian blur.')
    
    # Returning arguments
    return parser.parse_args()

size_image_x = 1280
# Global variables for threading
connected = False  # Timer for connection to server
robotMode = 0      # Starting mode of the robot - waiting for connexion
client_socket = socket.socket(socket.AF_INET, socket.SOCK_STREAM)  # Socket of client

# Threads functions
def trying_to_connect():
    global connected
    global client_socket
    # Waiting for an answer, program will wait at this line until answer comes
    data = client_socket.recv(1024)
    print("Received '"+str(data.decode())+"' from server.")
    # If there is an answer, then function will continue meaning connection is successful.
    # Otherwise, main() will end the program after given time.
    # It does not matter what the robot receive, this is just to confirm that the server
    # saved the robot IP for further messages.
    connected = True
    print("Successfully connected to server.")

def listen_for_server():
    global robotMode
    global client_socket
    # while not shutting down
    while robotMode!=-1:
        # reading buffer
        data = client_socket.recv(1024)
        print("Received '"+str(data.decode())+"' from server.")
        # changing mode depending on buffer
        if data.decode()=="Standby":
            robotMode = 0
        if data.decode()=="Search":
            robotMode = 1
        if data.decode()=="Leader":
            robotMode = 2
        if data.decode()=="Follower":
            robotMode = 3
        if data.decode()=="Shutdown":
            robotMode=-1
        print("Mode changed to "+str(robotMode)+": "+str(data.decode()))



# Main function
def main(args):
    # modes of the robot:
    # 0  = awaiting orders
    # 1  = searching
    # 2  = leader (following ball)
    # 3  = follower (searching and following leader)
    # -1 = shutdown
    global robotMode
    global connected
    global client_socket
    
    # Gathering parameters from parser
    low_threshold = args.lowThresh
    high_threshold = args.highThresh
    minDist = args.minDist
    param1 = args.param1
    param2 = args.param2
    minRad = args.minRad
    maxRad = args.maxRad
    gauss_rad = args.blurRad

    # Serial ports for communication with raspberry
    ser = serial.Serial('/dev/ttyUSB0')
    ser.reset_input_buffer()

    # Connecting socket to given address and port
    client_socket.connect((args.serverIP, args.serverPort))

    # Timer thread that will try to connect
    t_connect = threading.Thread(target=trying_to_connect)
    t_connect.start()
    # Number of seconds before abandoning connection (NB: This can be increased)
    seconds = 10
    # Waiting for connection (testing every seconds)
    while not connected and seconds>0:
        # Asking for connection
        client_socket.send(b"Request connection")
        # Waiting for one second
        time.sleep(1)
        seconds-=1
        # Testing with while if connected
        # The thread function will change the connected variable to True
        # and break the loop if it detect an answer from the server.

    # Failed to connect in given time
    if not connected:
        print('Failed to connect to {args.serverIP} on port {args.serverPort}.\nStopping program.')
        # return of main, ending program
        return

    # Else, connection successful
    # From this point, robot will listen to server to change its mode
    t_mode = threading.Thread(target=listen_for_server)
    t_mode.start()

    # Number of frames before telling server that ball is lost
    lost_frames = 5 # If changed, also change in robotmode 2
    # Previous values for intelligent selection of the ball
    circle = None
    # for faster computing, we will not take the square root of the sum of squares but just the sum (*2 for each axis)
    max_dist_allowed = 500 *2 # in pixels, dist of found centers from previous mean
    max_dist_for_dual = 100 *2 # in pixels, dist between centers if 2 circles and no previous info

    #Camera
    camera = PiCamera()
    camera.rotation = 180

    # matrix that will get input values
    frame=np.empty((720,1280,3),dtype=np.uint8)

    # main loop, depending on mode
    while robotMode!=-1:
        # Standby mode
        if robotMode==0:
            # Do nothing; waiting for orders
            continue
        
        # Searching mode
        if robotMode==1:
            # Strategy:
            # Turns around while making pauses to detect the ball

            # Capture frame
            camera.capture(frame,'rgb')

            # Convert to hsv
            hsv = cv2.cvtColor(frame, cv2.COLOR_RGB2HSV)

            # Construct a mask for the color "blue", then perform
            # a series of dilations and erosions to remove noise
            mask = cv2.inRange(hsv, low_threshold, high_threshold)
            mask = cv2.erode(mask, None, iterations=2)
            mask = cv2.dilate(mask, None, iterations=2)

            # Blurring to help hough transform detect circle from mask
            blurred = cv2.GaussianBlur(mask, (gauss_rad * 2 + 1, gauss_rad * 2 + 1), 0)

            # hough transform
            circles = cv2.HoughCircles(blurred, cv2.HOUGH_GRADIENT, 1, minDist, param1=param1, param2=param2, minRadius=minRad,
                               maxRadius=maxRad)

            
            # Test if a circle (ball)  has been found
            if circles is not None:
                # Send message to server and change mode if not changed already
                print("Found Ball.")
                client_socket.send(b"Found ball")
                # Change mode to waiting if server hasn't asked to change mode already
                if robotMode==1:
                    robotMode=0
                    client_socket.send(b"Returning to standby mode")
                    print("Changing mode to 0: Standby, and waiting for leader assignment.")
                # Telling the robot to stop turning on himself
                msg = ""
                ser.write(msg.encode('utf-8'))

            else:
                # Telling the robot to turn on himself
                msg = "right"
                ser.write(msg.encode('utf-8'))

            # End of searching mode
            continue

        # Leader mode
        if robotMode==2:
            # Strategy:
            # Follows the ball

            # Capture frame
            camera.capture(frame,'rgb')

            # Convert to hsv
            hsv = cv2.cvtColor(frame, cv2.COLOR_RGB2HSV)

            # Construct a mask for the color "blue", then perform
            # a series of dilations and erosions to remove noise
            mask = cv2.inRange(hsv, low_threshold, high_threshold)
            mask = cv2.erode(mask, None, iterations=2)
            mask = cv2.dilate(mask, None, iterations=2)

            # Blurring to help hough transform detect circle from mask
            blurred = cv2.GaussianBlur(mask, (gauss_rad * 2 + 1, gauss_rad * 2 + 1), 0)

            # hough transform
            circles = cv2.HoughCircles(blurred, cv2.HOUGH_GRADIENT, 1, minDist, param1=param1, param2=param2, minRadius=minRad,
                               maxRadius=maxRad)

            # Choose one circle among those found if any
            if circles is not None and circles[0][0].ndim == 1:
                lost_frames = 5 # Reset number of frames until lost
                circles = np.uint16(np.around(circles))
                nb_circles = len(circles[0,:])
                if nb_circles>1: # multiple circles found
                    # The chosen circle index
                    index_circle = -1

                    if circle is not None:
                        # we take the closest to the previous position, if it's not too far
                        dists = [((circle[0] - i[0]) + (circle[1] - i[1])) for i in circles[0, :]]
                        min_dist = dists[0]+1 # Force first iteration
                        index_i=0
                        for i in dists:
                            if i<max_dist_allowed and i<min_dist:
                                min_dist = i
                                index_circle = index_i
                            index_i+=1


                    else:
                        # No previous circles
                        if nb_circles==2:
                            # test if they are close to each other and take the mean, otherwise can't choose between the two
                            dist = (circles[0,0,0]-circles[0,1,0]) + (circles[0,0,1]-circles[0,1,1])
                            if dist<max_dist_for_dual:
                                circle = ((circles[0,0,:]+circles[0,1,:])/2).astype(int)
                                index_circle = None # Signals that circle has been assigned
                        else:
                            # mean of circles
                            mean_circle = circles[0,-1,:]
                            for i in range(nb_circles-1):
                                mean_circle += circles[0,i,:]
                            mean_circle = mean_circle/nb_circles

                            # taking index of closest (center and range)
                            min_dist = sum(circles[0,0,:]-mean_circle)+1
                            index_i = 0
                            for i in circles[0,:]:
                                dist = sum(i-mean_circle)
                                if dist<max_dist_allowed and dist<min_dist:
                                    min_dist = dist
                                    index_circle = index_i
                                index_i+=1

                    if index_circle is not None: # not already assigned
                        if index_circle==-1:
                            # this might be another circle or we moved the circle too far away
                            circle = None
                        else:
                            # Chosen circle in previous lines
                            circle=circles[0,index_circle]


                else: # Only one circle found
                    # compared it to last position
                    if circle is not None:
                        dist = (circle[0]-circles[0,0,0]) + (circle[1]-circles[0,0,1])
                        if dist<max_dist_allowed:
                            # Circle seems to be the same
                            circle = circles[0, 0]
                        else:
                            # this might be another circle or we moved the circle far away
                            circle = None

                    else:
                        # First circle found in some time
                        circle = circles[0,0]

            else:
                lost_frames-=1
                circle=None

            # Testing if robot has lost ball
            if lost_frames==0:
                # Send warning to server
                client_socket.send(b"Lost ball")
                # Returning to standy mode and waiting for orders
                robotMode = 0
                # Print information
                #client_socket.send(b"Returning to standby mode")
                print("Lost ball.\nReturning to standby mode.")

            # Constructing message to send to arduino
            msg = ''
            if circle is not None:
                # Computing order to give to the arduino
                # Position
                if circle[0]<500:
                    Angle=1
                elif circle[0]>780:
                    Angle=2
                else:
                    Angle=0

                if circle[2]>100:
                    Distance=2
                elif circle[2]<60:
                    Distance=1
                else:
                    Distance=0

                # Message
                if (Angle == 0):
                    if (Distance == 0):
                        # déjà ok  zone a définir pour que le robot se stabilise
                        msg = ''
                    elif (Distance == 1):
                        msg = "avance"
                    else:
                        msg = 'recul'
                elif (Angle == 1):
                    msg = "left"
                else:
                    msg = "right"
                
            # Give order to arduino
            ser.write(msg.encode('utf-8'))
            
            # End of leader mode
            continue

        # Follower mode
        if robotMode==3:
            # Searches for the leader then follows the leader (Using aruco tags)

            # Aruco definition
            aruco_dict = aruco . Dictionary_get ( aruco . DICT_6X6_250 )
            parameters = aruco . DetectorParameters_create ()
            cap = cv2 . VideoCapture (0)
            
            mtx = np.load("mtx.npy")
            dist = np.load("dist.npy")
            
            rows = 2#TODO : Select the right number of rows
            cols = 2 #TODO : Select the right number of columsq
            square_size = 4.75 #TODO : Set the square size in mm
            
            # Set the termination criteria for the corner sub-pixel algorithm
            criteria = (cv2.TERM_CRITERIA_MAX_ITER + cv2.TERM_CRITERIA_EPS, 30, 0.001)
            
            # Prepare the object points: (0,0,0), (1,0,0), (2,0,0), ..., (6,5,0). They are the same for all images
            objectPoints = np.zeros((rows * cols, 3), np.float32)
            objectPoints[:, :2] = np.mgrid[0:rows, 0:cols].T.reshape(-1, 2)*square_size
            a=4.75
            msf = ''
            while True :
                time.sleep(1)
                # Capture frame -by - frame
                _ , frame = cap . read ()
                # Our operations on the frame come here
                gray = cv2 . cvtColor ( frame , cv2 . COLOR_BGR2GRAY )
                # lists of ids and the corners belonging to each id 
                corners , ids , rejectedImgPoints = aruco . detectMarkers ( gray ,aruco_dict , parameters = parameters )
                # print ( ids )
                # print ( corners )
                gray = aruco . drawDetectedMarkers ( gray , corners )
                # Display the resulting frame
                cv2 . imshow ('frame ', gray )
                if cv2 . waitKey (1) & 0xFF == ord('q') :
                    break
                if(len(corners)>0):
                    tval,rvec,tvec=cv2.solvePnP(np.array([[-a/2, -a/2, 0],
                                                          [-a/2, a/2, 0],
                                                          [a/2, a/2, 0],
                                                          [a/2, -a/2, 0]])
                                                ,corners[0], mtx, dist)
                    #print(tvec)
                moyenne=(corners[0][0]+corners[0][1]+corners[0][2]+corners[0][3])/4
                position_centre = size_image_x/2-moyenne #if >0  right if not left 
                
                cap . release ()            # moyenne des 4 valeur de x corner, dif entre centre valeur obtient angle le sens dépend de droite ou gauche du centre 
                cv2 . destroyAllWindows ()
                
                if (position_centre < -50):
                    msg = 'right' 
                elif (position_centre > 50):                 
                    msg = 'left'
                elif (tvec[0][2]>25):
                    msg = 'avance'
                elif (tvec[0][2]<10):
                    msg = 'recul'
                else:
                    continue
                ser.write(msg.encode('utf-8'))    
            continue


    # Send command to arduino to not move the robot
    ser.reset_input_buffer() # Resetting command buffer
    ser.close() # closing serial connection
    # Send shutdown notice
    client_socket.send(b"Shutting down")
    print("Shutting down.")
    # Close client socket
    client_socket.close()
    # Close camera
    camera.stop_preview()
    camera.close()


if __name__ == "__main__":
    args = parse_opt()
    # Tests if a server has been specified
    if args.serverIP!="NOSERVER":
        main(args)
    # otherwise, doesn't start the program
    else:
        print("Please specify --serverIP")