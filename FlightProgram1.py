# Daniel S. Cohen

#

"""
Thesis Flight Code: Inv Kino code
"""

import rospy
from mavros_msgs.msg import State, PositionTarget
# from apriltag_ros.msg import AprilTagDetectionArray
from geometry_msgs.msg import PoseStamped, Point, Quaternion, TransformStamped, PoseArray, TwistStamped
from sensor_msgs.msg import CameraInfo, RegionOfInterest
import math
import numpy
from image_geometry import PinholeCameraModel
import time
# from darknet_ros_msgs.msg import BoundingBox,BoudingBoxes

from mavros_msgs.srv import CommandBool, SetMode
from std_msgs.msg import String
from tf.transformations import quaternion_from_euler

# from gazebo_ros_link_attacher.msg import Attach
from gazebo_ros_link_attacher.srv import Attach, AttachRequest, AttachResponse

from gazebo_msgs.srv import SpawnModel, SpawnModelRequest, SpawnModelResponse
from copy import deepcopy

from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint


global shutdown_robot


# our class
class OffbPosCtl:
    curr_drone_pose = PoseStamped()  # create the right structer of the msg for use
    waypointIndex = 0  # start with a way point index of 0
    distThreshold = 0.15  # meters how close would you like to be from the waypoit
    sim_ctr = 1
    arm_ready = True
    length_of_first_link = 0.15
    length_of_second_link = 0.15
    robot_tip_x_locaiton = 0
    robot_tip_y_location = 0
    joint_1_pos = 0
    joint_2_pos = 0
    camera = PinholeCameraModel()

    tree_x = -1.899290  # tree locaiton in meters
    tree_y = -3.340390# tree location in meters
    tree_z = 0.7 # tree location in meters

    des_pose = PoseStamped()
    # des_vel = TwistStamped()
    isReadyToFly = False
    attach = False
    prob_found = False

    shutdown_robot = False
    # location
    # x_vel = numpy.linspace(0.0, 0.1, 1000)
    # y_vel = numpy.linspace(0.1, 0.0, 1000)

    # get orientation of the des_robot
    orientation = quaternion_from_euler(0, 0, 3.14 / 2 + 3.14 / 8)
    orientation2 = quaternion_from_euler(0, 0, 3.14 / 2)
    orientation3 = quaternion_from_euler(0, 0, 2.7475)
    orientation4 = quaternion_from_euler(0, 0, -2.7475)
    orientation0 = quaternion_from_euler(0, 0, 0)
    orientation5 = quaternion_from_euler(0, 0, 0)

    orientation7 = quaternion_from_euler(0, 0, 2.7475)
    orientation8 = quaternion_from_euler(0, 0, 3.0)
    orientation9 = quaternion_from_euler(0, 0, 3.4)
    orientation10 = quaternion_from_euler(0, 0, 4.1)
    orientation11 = quaternion_from_euler(0, 0, 4.8)
    orientation12 = quaternion_from_euler(0, 0, 5.5)
    orientation13 = quaternion_from_euler(0, 0, 6.2)
    orientation14 = quaternion_from_euler(0, 0, 6.9)
    orientation15 = quaternion_from_euler(0, 0, 7.3)
    orientation16 = quaternion_from_euler(0, 0, 8.3)
    orientation17 = quaternion_from_euler(0, 0, 9.0)
    orientation18 = quaternion_from_euler(0, 0, 0)

    # Store the robots location for waypoint nav system, preloaded from our reaserch out the world map

    locations = numpy.matrix([[2, 0, 1, orientation2[0], orientation2[1], orientation2[2], orientation2[3]],
                              [2, 2, 1, orientation2[0], orientation2[1], orientation2[2], orientation2[3]],
                              [0, 2, 1, orientation2[0], orientation2[1], orientation2[2], orientation2[3]],
                              [-2, 0, 1, orientation2[0], orientation2[1], orientation2[2], orientation2[3]],
                              # end of fun box point 4

                              [-1.525730, -1.161221, 4.0, orientation0[0], orientation0[1], orientation0[2], orientation0[3]],
                              # start of scan and point4
                                ])

    # washer loc x,y,z 84.789988,-54.251096,17.83632
    def __init__(self):
        global shutdown_robot
        shutdown_robot = False
        rospy.init_node('offboard_test', anonymous=True)
        pose_pub = rospy.Publisher('/mavros/setpoint_position/local', PoseStamped, queue_size=10)
        # pose_setpoint_pub = rospy.Publisher('/mavros/setpoit_position/local geometry_msgs/PoseStamped', PoseStamped, queue_size=10)
        drone_pose_subscriber = rospy.Subscriber('/mavros/local_position/pose', PoseStamped,
                                                 callback=self.drone_pose_cb)
        # vel_pub = rospy.Publisher('mavros/local_raw/local', PositionTarget, queue_size=10)
        vel_pub = rospy.Publisher('/mavros/setpoint_velocity/cmd_vel', TwistStamped, queue_size=10)
        # vel more like pose
        rover_pose_subscriber = rospy.Subscriber('/mavros/local_position/pose', PoseStamped,
                                                 callback=self.rover_pose_cb)
        state_sub = rospy.Subscriber('/mavros/state', State, callback=self.drone_state_cb)
        # attach = rospy.Publisher('/attach', String, queue_size=10)

        # attach_pub = rospy.Publisher('/link_attacher_node/attach_models',Attach, queue_size=1)

        attach_srv = rospy.ServiceProxy('/link_attacher_node/attach', Attach)
        attach_srv.wait_for_service()
        rospy.loginfo("Created ServiceProxy to /link_attacher_node/attach")

        rospy.loginfo("Creating ServiceProxy to /link_attacher_node/detach")
        detach_srv = rospy.ServiceProxy('/link_attacher_node/detach', Attach)
        detach_srv.wait_for_service()
        rospy.loginfo("Created ServiceProxy to /link_attacher_node/detach")

        arm_pose_pub = rospy.Publisher('arm_controller/command', JointTrajectory, queue_size=1)

        NUM_UAV = 2
        mode_proxy = [None for i in range(NUM_UAV)]
        arm_proxy = [None for i in range(NUM_UAV)]
        x_vel = 0.0
        y_vel = 0.0
        z_vel = 0.0
        rot_speed = 0.0
        vel_moving = False

        # Comm for drones and get ready to take off the system, "preflight"
        for uavID in range(0, NUM_UAV):
            mode_proxy[uavID] = rospy.ServiceProxy(self.mavrosTopicStringRoot(uavID) + '/set_mode', SetMode)
            arm_proxy[uavID] = rospy.ServiceProxy(self.mavrosTopicStringRoot(uavID) + '/cmd/arming', CommandBool)

        rate = rospy.Rate(10)  # Hz
        rate.sleep()
        self.des_pose = self.copy_pose(self.curr_drone_pose)
        shape = self.locations.shape

        # Keep the uav running unitll we ros gets shut down or the robot triggers a shutdown
        while not rospy.is_shutdown() and shutdown_robot is False:
            print(self.sim_ctr, shape[0], self.waypointIndex)
            success = [None for i in range(NUM_UAV)]
            for uavID in range(0, NUM_UAV):
                try:
                    success[uavID] = mode_proxy[uavID](1, 'OFFBOARD')
                except rospy.ServiceException, e:
                    print ("mavros/set_mode service call failed: %s" % e)

            success = [None for i in range(NUM_UAV)]
            for uavID in range(0, NUM_UAV):
                rospy.wait_for_service(self.mavrosTopicStringRoot(uavID) + '/cmd/arming')

            for uavID in range(0, NUM_UAV):
                try:
                    success[uavID] = arm_proxy[uavID](True)
                except rospy.ServiceException, e:
                    print ("mavros1/set_mode service call failed: %s" % e)

            if self.waypointIndex is shape[0]:
                self.waypointIndex = 0
                self.sim_ctr += 1
            joint_1_pos = 0 # start the arm in the up pos
            joint_2_pos = 0

            if self.waypointIndex is 4:
                prob_found = False
                x_vel = 0.0
                y_vel = 0.0
                z_vel = 0.0
                rot_speed = 0.0
                vel_pub.publish(self.vel_move(x_vel, y_vel, z_vel, rot_speed))

                orientation = quaternion_from_euler(0, 0, 0)
                des_array = numpy.array(
                    [self.tree_x,self.tree_y+(self.length_of_first_link+self.length_of_second_link)/2,self.tree_z ,orientation[0], orientation[1], orientation[2], orientation[3]])
                des_local = self.set_desired_pose_local(des_array).position
                dist = math.sqrt((curr.x - des_local.x) * (curr.x - des_local.x) + (curr.y - des_local.y) * (
                        curr.y - des_local.y) + (curr.z - des_local.z) * (
                                         curr.z - des_local.z))
                print('dist = at waypoint 4', dist)

                while (dist > self.distThreshold):
                    # des = self.set_desired_pose().position

                    azimuth = math.atan2(self.curr_rover_pose.pose.position.y - self.curr_drone_pose.pose.position.y,
                                         self.curr_rover_pose.pose.position.x - self.curr_drone_pose.pose.position.x)
                    az_quat = quaternion_from_euler(0, 0, azimuth)

                    curr = self.curr_drone_pose.pose.position
                    dist = math.sqrt(
                        (curr.x - des_local.x) * (curr.x - des_local.x) + (curr.y - des_local.y) * (
                                    curr.y - des_local.y) + (curr.z - des_local.z) * (
                                curr.z - des_local.z))

                    print('dist = ', dist)
                    if (curr.x < des_local.x):
                        x_vel = 0.2
                        z_vel = -0.2
                        rot_speed = 0.0
                    else:
                        x_vel = -0.2

                    if (curr.y < des_local.y):

                        y_vel = 0.2
                        z_vel = -0.2
                        rot_speed = 0.0

                    else:
                        y_vel = -0.2
                        z_vel = -0.2

                    if (curr.z < des_local.z):
                        z_vel = 0.2

                    vel_pub.publish(self.vel_move(x_vel, y_vel, z_vel, rot_speed))

                    #rospy.init_node('arm_controller_node')

                    rospy.sleep(0.1)

                    msg = JointTrajectory()
                    msg.joint_names = ['base_link1_joint', 'base_link2_joint']
                    points = JointTrajectoryPoint()

                    if (self.arm_ready==True):
                        self.arm_ready=False
                        joint_1_pos = joint_1_pos + 0.785
                        joint_2_pos = joint_2_pos + 0.785
                        points.positions = [joint_1_pos, joint_2_pos]
                        points.time_from_start = rospy.Time(0.1)
                        msg.points = [points]

                        arm_pose_pub.publish(msg)
                        print(joint_1_pos,joint_2_pos)

                        # foward kinimatics
                        x_tip = self.length_of_first_link*math.cos(joint_1_pos) + self.length_of_second_link*math.cos(joint_1_pos+joint_2_pos)
                        y_tip = self.length_of_first_link*math.sin(joint_1_pos) + self.length_of_second_link*math.sin(joint_1_pos+joint_2_pos)

                        print("x_tip",x_tip,"y_tip",y_tip)

                        top  = x_tip * x_tip + y_tip* y_tip - self.length_of_first_link * self.length_of_first_link-self.length_of_second_link * self.length_of_second_link

                        bot = 2 * self.length_of_first_link * self.length_of_second_link

                        #print("top", top, "bot", bot)

                        q2 = math.acos(top/bot)

                        q1 = (math.atan(x_tip/y_tip))-math.atan((self.length_of_first_link+self.length_of_second_link*math.cos(q2))/(self.length_of_second_link*math.sin(q2)))

                        print("q1",-q1,"q2",q2)

                    print(curr.x - des_local.x)
                    print(curr.y - des_local.y)
                    print("curr x",curr.x)
                    print("curr y", curr.y)
                    print("uav des loc")
                    print(des_local.y-y_tip)
                    print(des_local.z-x_tip)
                    print("arm tip curr")
                    print(y_tip+curr.y)
                    print(x_tip+curr.x)
                    print("arm tip des")
                    print(-des_local.y-y_tip + curr.y)
                    print(-des_local.z-x_tip + curr.z)





                    print("x_tip", x_tip, "y_tip", y_tip)

                    if (self.length_of_first_link+self.length_of_second_link)/2 >=dist:
                        x_tip_des = math.fabs(-des_local.z - x_tip + curr.z)
                        y_tip_des = math.fabs(-des_local.y - y_tip + curr.y)


                        print(x_tip_des,y_tip_des)

                        top = x_tip_des * x_tip_des + y_tip_des * y_tip_des - self.length_of_first_link * self.length_of_first_link - self.length_of_second_link * self.length_of_second_link

                        bot = 2 * self.length_of_first_link * self.length_of_second_link

                        print("top", top, "bot", bot)

                        q2 = math.acos(top / bot)

                        q1 = (math.atan(x_tip_des / y_tip_des)) - math.atan(
                            (self.length_of_first_link + self.length_of_second_link * math.cos(q2)) / (
                                        self.length_of_second_link * math.sin(q2)))

                        print("q1", -q1, "q2", q2)

                        msg = JointTrajectory()
                        msg.joint_names = ['base_link1_joint', 'base_link2_joint']
                        points = JointTrajectoryPoint()

                        joint_1_pos = -q1
                        joint_2_pos = q2
                        points.positions = [joint_1_pos, joint_2_pos]
                        points.time_from_start = rospy.Time(0.01)
                        msg.points = [points]

                        arm_pose_pub.publish(msg)
                        print(joint_1_pos, joint_2_pos)


                    #var = False
                    if dist <= (self.length_of_first_link+self.length_of_second_link)/2: #self.distThreshold:
                        print("got it")
                        req = AttachRequest()
                        req.model_name_1 = "iris_1"
                        req.link_name_1 = "link2"
                        req.model_name_2 = "small_base_tree2"#"demo_joint_stiffness_hev" #"small_base_tree2"
                        req.link_name_2 = "link"#"link_low_stiffness" #"link"
                        attach_srv.call(req)
                        prob_found = True
                        vel_moving = False

                        orientation = quaternion_from_euler(0, 0, 0)
                        des_array = numpy.array(
                            [self.tree_x, self.tree_y-1,self.tree_z , orientation[0], orientation[1], orientation[2],

                             orientation[3]])
                        des_local = self.set_desired_pose_local(des_array).position
                        dist = math.sqrt((curr.x - des_local.x) * (curr.x - des_local.x) + (curr.y - des_local.y) * (
                                curr.y - des_local.y) + (curr.z - des_local.z) * (
                                                 curr.z - des_local.z))

                        print('dist2 = ', dist)
                        while (dist > self.distThreshold):
                            # des = self.set_desired_pose().position

                            azimuth = math.atan2(
                                self.curr_rover_pose.pose.position.y - self.curr_drone_pose.pose.position.y,
                                self.curr_rover_pose.pose.position.x - self.curr_drone_pose.pose.position.x)
                            az_quat = quaternion_from_euler(0, 0, azimuth)

                            curr = self.curr_drone_pose.pose.position
                            dist = math.sqrt(
                                (curr.x - des_local.x) * (curr.x - des_local.x) + (curr.y - des_local.y) * (
                                        curr.y - des_local.y) + (curr.z - des_local.z) * (
                                        curr.z - des_local.z))

                            print(des_local) # dont think that its pubing the vel

                            if (curr.x < des_local.x):
                                x_vel = 0.2
                                z_vel = -0.2
                                rot_speed = 0.0
                            else:
                                x_vel = -0.2

                            if (curr.y < des_local.y):

                                y_vel = 0.2
                                z_vel = -0.2
                                rot_speed = 0.0

                            else:
                                y_vel = -0.2
                                z_vel = -0.2

                            if (curr.z < des_local.z):
                                z_vel = 0.2


                            vel_pub.publish(self.vel_move(x_vel, y_vel, z_vel, rot_speed))





                            #pose_pub.publish(self.des_pose)

                        # self.waypointIndex += 1

                rate.sleep()

            if self.waypointIndex is 20 and vel_moving is False:
                print("way point 20")

                x_vel = 0.0
                y_vel = 0.0
                z_vel = 0.0
                rot_speed = 0.0
                vel_pub.publish(self.vel_move(x_vel, y_vel, z_vel, rot_speed))
                print("moving to target using the vel with rotate")
                print("MOVING........")
                vel_moving = False
                prob_ready_for_deployment = True
                x_vel = 0.0
                y_vel = 0.0
                z_vel = 0.0
                rot_speed = 0.0
                # vel_pub.publish(self.vel_move(x_vel, y_vel, z_vel, rot_speed))

                orientation = quaternion_from_euler(0, 0, 0)
                des_array = numpy.array(
                    [84.789988, -54.251096, 19.2, orientation[0], orientation[1], orientation[2], orientation[3]])

                des_local = self.set_desired_pose_local(des_array).position
                dist = math.sqrt((curr.x - des_local.x) * (curr.x - des_local.x) + (curr.y - des_local.y) * (
                        curr.y - des_local.y) + (curr.z - des_local.z) * (
                                         curr.z - des_local.z))
                count = 0

                ##84.789988,-54.251096,17.83632
                while (prob_ready_for_deployment is True and dist > self.distThreshold and count is 0):
                    # des = self.set_desired_pose().position
                    des_local = self.set_desired_pose_local(des_array).position
                    azimuth = math.atan2(self.curr_rover_pose.pose.position.y - self.curr_drone_pose.pose.position.y,
                                         self.curr_rover_pose.pose.position.x - self.curr_drone_pose.pose.position.x)
                    az_quat = quaternion_from_euler(0, 0, azimuth)

                    curr = self.curr_drone_pose.pose.position
                    dist = math.sqrt(
                        (curr.x - des_local.x) * (curr.x - des_local.x) + (curr.y - des_local.y) * (
                                curr.y - des_local.y) + (curr.z - des_local.z) * (
                                curr.z - des_local.z))

                    print('dist = ', dist, "ready for depl", prob_ready_for_deployment)


                    if (curr.x < des_local.x):
                        x_vel = 0.1
                        z_vel = -0.2
                        rot_speed = 0.0
                    else:
                        x_vel = -0.1

                    if (curr.y < des_local.y):

                        y_vel = 0.1
                        z_vel = -0.2
                        rot_speed = 0.0

                    else:
                        y_vel = -0.1
                        z_vel = -0.2

                    if (curr.z < des_local.z):
                        z_vel = 0.2
                    vel_pub.publish(self.vel_move(x_vel, y_vel, z_vel, rot_speed))

                    if dist < self.distThreshold:
                        print("got it")
                        req = AttachRequest()
                        req.model_name_1 = "iris"
                        req.link_name_1 = "base_link"
                        req.model_name_2 = "sample_probe"
                        req.link_name_2 = "base_link"
                        detach_srv.call(req)
                        prob_ready_for_deployment = False
                        vel_moving = True
                        count = count + 1

                rate.sleep()

            if self.waypointIndex is 24 and vel_moving is True:
                print("way point 24")

                x_vel = 0.0
                y_vel = 0.0
                z_vel = 0.0
                rot_speed = 0.0
                vel_pub.publish(self.vel_move(x_vel, y_vel, z_vel, rot_speed))
                print(" moving to target using the vel ")
                print("MOVING........ to way point 14")
                vel_moving = False
                landed = False
                x_vel = 0.0
                y_vel = 0.0
                z_vel = 0.0
                rot_speed = 0.0
                vel_pub.publish(self.vel_move(x_vel, y_vel, z_vel, rot_speed))

                orientation = quaternion_from_euler(0, 0, 3.14 / 2)
                des_array = numpy.array(
                    [12.62140, -65.05, -3.8, orientation[0], orientation[1], orientation[2], orientation[3]])

                while (landed is False):
                    # des = self.set_desired_pose().position
                    des_local = self.set_desired_pose_local(des_array).position
                    azimuth = math.atan2(self.curr_rover_pose.pose.position.y - self.curr_drone_pose.pose.position.y,
                                         self.curr_rover_pose.pose.position.x - self.curr_drone_pose.pose.position.x)
                    az_quat = quaternion_from_euler(0, 0, azimuth)

                    curr = self.curr_drone_pose.pose.position
                    dist = math.sqrt(
                        (curr.x - des_local.x) * (curr.x - des_local.x) + (curr.y - des_local.y) * (
                                curr.y - des_local.y) + (curr.z - des_local.z) * (
                                curr.z - des_local.z))
                    if (curr.x < des_local.x):
                        x_vel = 0.1
                        z_vel = -0.2
                        rot_speed = 0.0
                    else:
                        x_vel = -0.1
                        z_vel = -0.2

                    if (curr.y < des_local.y):

                        y_vel = 0.1
                        z_vel = -0.2
                        rot_speed = 0.0

                    # if(curr.z < des_local.z):
                    # z_vel = 0.2

                    else:
                        y_vel = -0.1
                        z_vel = -0.2

                    vel_pub.publish(self.vel_move(x_vel, y_vel, z_vel, rot_speed))
                    print('dist = ', dist, "Robot landed(T/F)", landed)
                    print("In the truckbed (T/F)", dist < 1.5)
                    if dist < self.distThreshold:
                        print("landed")
                        landed = True
                        vel_moving = False
                        shutdown_robot = True

                        success = [None for i in range(NUM_UAV)]
                        for uavID in range(0, NUM_UAV):
                            rospy.wait_for_service(self.mavrosTopicStringRoot(uavID) + '/cmd/arming')

                        for uavID in range(0, NUM_UAV):
                            try:
                                success[uavID] = arm_proxy[uavID](False)
                                print ("mavros1/set_mode service call was successful", success[uavID])

                            except rospy.ServiceException, e:
                                print ("mavros1/set_mode service call failed: %s" % e)
                        # drone is on the robot ready to move to new map for scanning

                rate.sleep()

            if self.isReadyToFly and shutdown_robot is False:
                des = self.set_desired_pose().position
                azimuth = math.atan2(self.curr_rover_pose.pose.position.y - self.curr_drone_pose.pose.position.y,
                                     self.curr_rover_pose.pose.position.x - self.curr_drone_pose.pose.position.x)
                az_quat = quaternion_from_euler(0, 0, azimuth)

                curr = self.curr_drone_pose.pose.position
                dist = math.sqrt(
                    (curr.x - des.x) * (curr.x - des.x) + (curr.y - des.y) * (curr.y - des.y) + (curr.z - des.z) * (
                            curr.z - des.z))
                if dist < self.distThreshold:
                    self.waypointIndex += 1

            pose_pub.publish(self.des_pose)
            rate.sleep()

    def mavrosTopicStringRoot(self, uavID=0):
        mav_topic_string = '/mavros/'
        return mav_topic_string

    def set_desired_pose(self):
        self.des_pose.pose.position.x = self.locations[self.waypointIndex, 0]
        self.des_pose.pose.position.y = self.locations[self.waypointIndex, 1]
        self.des_pose.pose.position.z = self.locations[self.waypointIndex, 2]
        self.des_pose.pose.orientation.x = self.locations[self.waypointIndex, 3]
        self.des_pose.pose.orientation.y = self.locations[self.waypointIndex, 4]
        self.des_pose.pose.orientation.z = self.locations[self.waypointIndex, 5]
        self.des_pose.pose.orientation.w = self.locations[self.waypointIndex, 6]
        if self.locations[self.waypointIndex, :].sum() == 0:
            self.des_pose.pose.position.x = self.curr_rover_pose.pose.position.x
            self.des_pose.pose.position.y = self.curr_rover_pose.pose.position.y
            self.des_pose.pose.position.z = max(self.curr_rover_pose.pose.position.z, 10)
            orientation = quaternion_from_euler(0, 0, 3.14 / 2)
            self.des_pose.pose.orientation = Quaternion(orientation[0], orientation[1], orientation[2], orientation[3])
        return self.des_pose.pose

    def set_desired_pose_local(self, array_input):
        # self.des_pose= PoseStamped()

        self.des_pose.pose.position.x = array_input[0]
        self.des_pose.pose.position.y = array_input[1]
        self.des_pose.pose.position.z = array_input[2]
        self.des_pose.pose.orientation.x = array_input[3]
        self.des_pose.pose.orientation.y = array_input[4]
        self.des_pose.pose.orientation.z = array_input[5]
        self.des_pose.pose.orientation.w = array_input[6]
        orientation = quaternion_from_euler(0, 0, 3.14 / 2)
        self.des_pose.pose.orientation = Quaternion(orientation[0], orientation[1], orientation[2], orientation[3])
        return self.des_pose.pose

    def copy_pose(self, pose):
        pt = pose.pose.position
        quat = pose.pose.orientation
        copied_pose = PoseStamped()
        copied_pose.header.frame_id = pose.header.frame_id
        copied_pose.pose.position = Point(pt.x, pt.y, pt.z)
        copied_pose.pose.orientation = Quaternion(quat.x, quat.y, quat.z, quat.w)
        return copied_pose

    def vel_move(self, x, y, z, rot_speed):
        des_vel = TwistStamped()
        des_vel.header.frame_id = 'world'
        des_vel.header.stamp = rospy.Time.from_sec(time.time())
        # des_vel.coordinate_frame = 8
        # des_vel.type_mask = 3527
        des_vel.twist.linear.x = x
        des_vel.twist.linear.y = y
        des_vel.twist.linear.z = z
        des_vel.twist.angular.z = rot_speed
        return des_vel

    def drone_pose_cb(self, msg):
        self.curr_drone_pose = msg

    def rover_pose_cb(self, msg):
        self.curr_rover_pose = msg

    def drone_state_cb(self, msg):
        print msg.mode
        if (msg.mode == 'OFFBOARD'):
            self.isReadyToFly = True
            print "readyToFly"


if __name__ == "__main__":
    OffbPosCtl()


