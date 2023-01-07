import time
import numpy as np
import math

import rclpy
from rclpy.node import Node

from geometry_msgs.msg import Twist
from sensor_msgs.msg import Image

from tello_msgs.srv import TelloAction
from rclpy.qos import qos_profile_sensor_data
from cv_bridge import CvBridge, CvBridgeError  # Package to convert between ROS and OpenCV Images
import cv2  # OpenCV library
import cv2.aruco as aruco

from Pid import PID

# Define the chess board rows and columns
square_size = 200 # Set the square size in mm

# the object frame is defined at the center of the marker
objectPoints = np.array([[-square_size/2, square_size/2, 0],
                         [square_size/2, square_size/2, 0],
                         [square_size/2, -square_size/2, 0],
                         [-square_size/2, -square_size/2, 0]])

class ActionManager(Node):
    def __init__(self):
        super().__init__('controller_manager')
        self.cli = self.create_client(TelloAction, '/drone1/tello_action')
        while not self.cli.wait_for_service(timeout_sec=1.0):
            self.get_logger().info('service not available, waiting again...')
        self.req = TelloAction.Request()

    def ask_for_takeoff(self):
        self.req.cmd = "takeoff"
        self.future = self.cli.call_async(self.req)

    def ask_for_landing(self):
        self.req.cmd = "land"
        self.future = self.cli.call_async(self.req)


class Controller(Node):

    def __init__(self):
        super().__init__('controller')

        ### CREATE THE PUBLISHER FOR COMMAND VELOCITY
        self.cmd_vel_publisher_ = self.create_publisher(Twist, '/drone1/cmd_vel', 10)
        timer_period = 0.5  # seconds
        self.timer = self.create_timer(timer_period, self.cmd_vel_loop)
        # Init desired speed
        self.vx = 0.0
        self.vy = 0.0
        self.vz = 0.0
        self.v_yaw = 0.0

        ### CREATE THE SUBSCRIBER FOR IMAGE RECEPTION
        qos_policy = rclpy.qos.QoSProfile(reliability=rclpy.qos.ReliabilityPolicy.BEST_EFFORT,
                                          history=rclpy.qos.HistoryPolicy.KEEP_LAST,
                                          depth=1)
        self.image_subscription_ = self.create_subscription(Image, '/drone1/image_raw', self.image_callback, qos_profile=qos_policy)
        self.display_image = True
        # Used to convert between ROS and OpenCV images
        self.br = CvBridge()

    def cmd_vel_loop(self):
        """
        I'm used to publish every 0.5 seconds the velocity command.
        You don't need to modify me, just modify vx, vy and v_yaw when you need it.
        """
        msg = Twist()
        msg.linear.x = self.vx
        msg.linear.y = self.vy
        msg.linear.z = self.vz
        msg.angular.z = self.v_yaw
        self.cmd_vel_publisher_.publish(msg)

    def image_callback(self, data):
        """
        I'm used to get camera's image.
        I can set the desired velocity according to the aruco detection
        """
        try:
            cv_image = self.br.imgmsg_to_cv2(data)
            # Our operations on the frame come here
            gray = cv2.cvtColor(cv_image, cv2.COLOR_BGR2GRAY)
            aruco_dict = aruco.Dictionary_get(aruco.DICT_6X6_1000)
            parameters = aruco.DetectorParameters_create()

            # lists of ids and the corners beloning to each id
            corners, ids, rejectedImgPoints = aruco.detectMarkers(gray, aruco_dict, parameters=parameters)

            gray = aruco.drawDetectedMarkers(gray, corners)

            if len(corners) > 0:
            
                print("TAG SEEN %s"%ids)
                #TODO : Here, write your controller
                #From the corners information, set the right command speed
                #Like self.vx, self.vy, self.v_yaw, to control the UAV
                
                # retrieve camera intrinsic matrix from topic /CameraInfo
                mtx = np.array([[921.9938565545156, 0.0 ,480.5],[0,921.9938565545156,360.5],[0,0,1]])
                
                # retrive camera distortion model from the same topic
                dist = np.zeros((5,1))
                # use Pnp Solver
                retval, rvec, tvec =cv2.solvePnP(objectPoints,corners[0],mtx, dist)
                
                # transform the roation vector in a rotation matrix
                R = cv2.Rodrigues(rvec)[0]
                # Determine the Euler angles
                roll = 180*math.atan2(-R[2][1], R[2][2])/np.pi
                pitch = 180*math.atan2(-R[2][0],math.sqrt(R[2][1]**2 + R[2][2]**2))/np.pi
                yaw = 180*math.atan2(-R[1][0], R[0][0])/np.pi
                rot_params= [roll,pitch,yaw]
                # print translation
                # print("translation vector in the camera frame : ",tvec)
                # print rotation

                # vy is for horizontal translation
                # but in tvec it's 1st element for horizontal
                
                self.vy = -0.0005* tvec[0][0]      #ça va a gauche 
                
                # vz is for vertical translation
                # but in tvec it's 2nd element for vertical

                self.vz = -0.0005* tvec[1][0]

                # vx is for depth translation
                # but in tvec it's 3rd element for depth


                pid = PID(0.01,0,0.01)

                self.v_yaw = - pid.loop(rot_params[1])

                # if (rot_params[1] <= 10):
                #     self.v_yaw =  0.001 * abs(rot_params[1])
                # elif (rot_params[1] > -10): 
                #     self.v_yaw = -0.001 * abs(rot_params[1])
                # else:
                #     print("uiiiii")
                #     self.v_yaw = 0.0


                # vx is for vertical translation
                # but in tvec it's 2nd element for vertical

                # print(rot_params)

                if (tvec[2][0] <= 2000):
                    self.vx = -min(0.0005* abs(tvec[2][0] - 2000) , 0.5)
                else : 
                    self.vx = min(0.0005* abs(tvec[2][0] - 2000) , 0.5)



		 # Get the camera pose
		 #R_cam = R.T
		 #tvec_cam = -R@tvec
		 #print(R_cam)
		 #print(tvec_cam)
                
            else:
                print("No tag seen..")
                #The UAV stay in the same pose, it doesn't move
                self.vx = 0.0
                self.vy = 0.0
                self.vz = 0.0
                self.v_yaw = 0.0 

            # Display the resulting frame
            if self.display_image:
                cv2.imshow('frame', gray)
                cv2.waitKey(3)

        except CvBridgeError as e:
            pass


def main(args=None):
    rclpy.init(args=args)

    # Wait for take off service to be ready
    action_manager = ActionManager()
    action_manager.ask_for_takeoff()
    ready_to_continue_mission = False

    # Try to takeoff, wait for the return of the service
    while rclpy.ok():
        rclpy.spin_once(action_manager)
        if action_manager.future.done():
            try:
                response = action_manager.future.result()
                if response.rc == 1:  # OK
                    print("Take off is a success !")
                    ready_to_continue_mission = True
                else:  # NOT OK
                    print("Something is wrong with the takeoff. LANDING NOW")
            except Exception as e:
                action_manager.get_logger().info("Service call failed %r" % (e,))
            break

    if ready_to_continue_mission:
        controller = Controller()
        try:
            while rclpy.ok():
                rclpy.spin_once(controller)
        except KeyboardInterrupt:
            print("Stopping the control. Ask for landing.")
        controller.destroy_node()

    # ASK FOR LANDING
    action_manager.ask_for_landing()
    while rclpy.ok():
        rclpy.spin_once(action_manager)
        if action_manager.future.done():
            try:
                response = action_manager.future.result()
                if response.rc == 1:  # OK
                    print("Landing is a success !")
                    break  # Only if landing is ok
            except Exception as e:
                action_manager.get_logger().info("Service call failed %r" % (e,))
            break

    rclpy.shutdown()


if __name__ == '__main__':
    main()
