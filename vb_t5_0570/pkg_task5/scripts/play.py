#! /usr/bin/env python

import rospy
import moveit_commander
import moveit_msgs.msg
import geometry_msgs.msg
import actionlib
import rospkg
import cv2
import yaml
import os
import math
import time
import sys
import copy
from moveit_commander.conversions import pose_to_list
from pkg_vb_sim.srv import vacuumGripper,vacuumGripperRequest
from pkg_vb_sim.srv import conveyorBeltPowerMsg,conveyorBeltPowerMsgRequest
from std_srvs.srv import Empty
import threading
import requests
from sensor_msgs.msg import Image
from cv_bridge import CvBridge, CvBridgeError
from pkg_ros_iot_bridge.msg import msgIotRosAction      # Message Class that is used by ROS Actions internally
from pkg_ros_iot_bridge.msg import msgIotRosGoal        # Message Class that is used for Goal Messages
from pkg_ros_iot_bridge.msg import msgIotRosResult      # Message Class that is used for Result Messages
from pkg_ros_iot_bridge.msg import msgIotRosFeedback    # Message Class that is used for Feedback Messages    
from pyzbar.pyzbar import decode
from pkg_ros_iot_bridge.msg import msgMqttSub           # Message Class for MQTT Subscription Messages
from pkg_task1.msg import myActionMsgResult
import json 
from pyiot import iot  
from Queue import PriorityQueue
from datetime import datetime, timedelta
import time

from datetime import datetime, timedelta
global a,b,c,d,e,f,g,h,i,res
a=1
b=1
c=1
d=1
e=1
f=1
g=1
h=1
i=1





class Ur5Moveit:

    # Constructor
    def __init__(self, arg_robot_name):

        rospy.init_node('node_moveit_eg7', anonymous=True)

        self._robot_ns = '/'  + arg_robot_name
        self._planning_group = "manipulator"
        
        self._commander = moveit_commander.roscpp_initialize(sys.argv)
        self._robot = moveit_commander.RobotCommander(robot_description= self._robot_ns + "/robot_description", ns=self._robot_ns)
        self._scene = moveit_commander.PlanningSceneInterface(ns=self._robot_ns)
        self._group = moveit_commander.MoveGroupCommander(self._planning_group, robot_description= self._robot_ns + "/robot_description", ns=self._robot_ns,wait_for_servers=0)
        self._display_trajectory_publisher = rospy.Publisher( self._robot_ns + '/move_group/display_planned_path', moveit_msgs.msg.DisplayTrajectory, queue_size=1)
        self._exectute_trajectory_client = actionlib.SimpleActionClient( self._robot_ns + '/execute_trajectory', moveit_msgs.msg.ExecuteTrajectoryAction)
        self._exectute_trajectory_client.wait_for_server()

        self._planning_frame = self._group.get_planning_frame()
        self._eef_link = self._group.get_end_effector_link()
        self._group_names = self._robot.get_group_names()
        self._box_name = 'Box_0'
        self.x=""
        self.queue = []
        self.city_queue = []
        self.orderid_queue = []
        # Attribute to store computed trajectory by the planner 
        self._computed_plan = ''

        # Current State of the Robot is needed to add box to planning scene
        self._curr_state = self._robot.get_current_state()

        rospy.loginfo(
            '\033[94m' + "Planning Group: {}".format(self._planning_frame) + '\033[0m')
        rospy.loginfo(
            '\033[94m' + "End Effector Link: {}".format(self._eef_link) + '\033[0m')
        rospy.loginfo(
            '\033[94m' + "Group Names: {}".format(self._group_names) + '\033[0m')


        rp = rospkg.RosPack()
        self._pkg_path = rp.get_path('pkg_moveit_examples')
        self._file_path = self._pkg_path + '/config/saved_trajectories/'
        rospy.loginfo( "Package Path: {}".format(self._file_path) )


        #rospy.loginfo('\033[94m' + " >>> Ur5Moveit init done." + '\033[0m')

        # Initialize the Action Server
        self._as = actionlib.ActionServer('/action_iot_ros',
                                          msgIotRosAction,
                                          self.on_goal,
                                          self.on_cancel,
                                          auto_start=False)
        self.bridge = CvBridge()
        self.image_sub = rospy.Subscriber("/eyrc/vb/camera_1/image_raw", Image,self.call,queue_size=1, buff_size = 2**28)    


        # Initialize Action Client
        self._ac = actionlib.ActionClient('/action_iot_ros',
                                          msgIotRosAction)
        
        # Dictionary to Store all the goal handels
        self._goal_handles = {}

        # Store the MQTT Topic on which to Publish in a variable
        param_config_iot = rospy.get_param('config_iot')
        self._config_mqtt_pub_topic = param_config_iot['mqtt']['topic_pub']

        # Wait for Action Server that will use the action - '/action_iot_ros' to start
        self._ac.wait_for_server()
        rospy.loginfo("Action server up, we can send goals.")

        self.already_called1=False
        self.already_called2=False
        self.already_called3=False
        self.already_called4=False
        self.already_called5=False
        self.already_called6=False
        self.already_called7=False
        self.already_called8=False
        self.already_called9=False
        self._cost=""
        self.prior=""
        self.order_id=""
        self.city=""
        self.x=""
        self.prior=""
        self.qty=""
        self._cost=""
        self.res= {}
    # This function will be called when there is a change of state in the Action Client State Machine
    def on_transition(self, goal_handle):
        
        # from on_goal() to on_transition(). goal_handle generated by send_goal() is used here.
        
        result = msgIotRosResult()

        index = 0
        for i in self._goal_handles:
            if self._goal_handles[i] == goal_handle:
                index = i
                break

        rospy.loginfo("Transition Callback. Client Goal Handle #: " + str(index))
        rospy.loginfo("Comm. State: " + str(goal_handle.get_comm_state()) )
        rospy.loginfo("Goal Status: " + str(goal_handle.get_goal_status()) )
        
        # Comm State - Monitors the State Machine of the Client which is different from Server's
        # Comm State = 2 -> Active
        # Comm State = 3 -> Wating for Result
        # Comm State = 7 -> Done
        
        # if (Comm State == ACTIVE)
        if goal_handle.get_comm_state() == 2:
            rospy.loginfo(str(index) + ": Goal just went active.")
        
        # if (Comm State == DONE)
        if goal_handle.get_comm_state() == 7:
            rospy.loginfo(str(index) + ": Goal is DONE")
            rospy.loginfo(goal_handle.get_terminal_state())
            
            # get_result() gets the result produced by the Action Server
            result = goal_handle.get_result()
            rospy.loginfo(result.flag_success)

            if (result.flag_success == True):
                rospy.loginfo("Goal successfully completed. Client Goal Handle #: " + str(index))
            else:
                rospy.loginfo("Goal failed. Client Goal Handle #: " + str(index))
    def sheetpush_dispatch(argv,arg_orderid,city,arg_item,arg_prior,arg_quantity,arg_cost,arg_status,disp_date_time):
        ur5= Ur5Moveit(sys.argv[1])
        dict_payload1 = {
        "Team id" : "vb_0570",
        "Unique Id" : "kanishka",
        "Order ID": arg_orderid,
        "City":city  ,
        
        "Item":arg_item ,             
        "Priority": arg_prior,
        "Dispatch Quantity": arg_quantity,
        "Cost":arg_cost,
        "Dispatch Status":arg_status ,
        "Dispatch Date and Time":disp_date_time ,#self.get_time_str()
        
        }
        str_payload1 = json.dumps(dict_payload1)
        ur5.send_goal("http","pubd","NA",str_payload1)

    def sheetpush_incoming(argv,arg_orderid,arg_time,arg_item,arg_prior,arg_quantity,city,lon,lat,arg_cost):
        ur5= Ur5Moveit(sys.argv[1])
        dict_payload = {
        "Team id" : "vb_0570",
        "Unique Id" : "kanishka",
        "Order ID": arg_orderid,
        "Order Time":arg_time,
        "Item":arg_item ,             
        "Priority": arg_prior,
        "Order Quantity": arg_quantity,
        "City":city  ,
        "Longitude":lon ,
        "Latitude":lat ,
        "Cost":arg_cost,
        }
        str_payload = json.dumps(dict_payload)
        ur5.send_goal("http","pubi","NA",str_payload)
    """
    def get_time_str(self):
        timestamp = int(time.time())
        value = datetime.datetime.fromtimestamp(timestamp)
        str_time = value.strftime('%Y-%m-%d %H:%M:%S')

        return str_time
    """

    # This function is used to send Goals to Action Server
    def send_goal(self, arg_protocol, arg_mode, arg_topic, arg_message):
        # Create a Goal Message object



        goal = msgIotRosGoal()

        goal.protocol = arg_protocol
        goal.mode = arg_mode
        goal.topic = arg_topic
        goal.message = arg_message
        

        rospy.loginfo("Send goal.")
        
        # self.on_transition - It is a function pointer to a function which will be called when 
        #                       there is a change of state in the Action Client State Machine
        goal_handle = self._ac.send_goal(goal,
                                         self.on_transition,
                                         None)

        return goal_handle  



    




    def call(self,data):                     #callback function for getting values from 2D camera
        #def ur5moveit():
        #    pass
        global color00,color01,color02,color10,color11,color12,color20,color21,color22
        


        try:
          cv_image = self.bridge.imgmsg_to_cv2(data, "bgr8")
        except CvBridgeError as e:
          rospy.logerr(e)

        (rows,cols,channels) = cv_image.shape
        
        image = cv_image

        
        


        barcodes = decode(image)

        # loop over the detected barcodes
        for barcode in barcodes:
            # extract the bounding box location of the barcode and draw the
            # bounding box surrounding the barcode on the image
            (x, y, w, h) = barcode.rect
            cv2.rectangle(image, (x, y), (x + w, y + h), (0, 0, 255), 2)

            # the barcode data is a bytes object so if we want to draw it on
            # our output image we need to convert it to a string first
            barcodeData = barcode.data.decode("utf-8")
            barcodeType = barcode.type
            left=barcode.rect.left
            top=barcode.rect.top
            #if left==128 and top==315:
                
                
                
                
            if left==502 and top==315:
                color02=barcodeData
            if left==316 and top==316:
                color01=barcodeData
            if left==316 and top==496:
                color11=barcodeData
            if left==502 and top==495:
                color12=barcodeData
            if left==128 and top==643:
                color20=barcodeData
            if left==315 and top==643:
                color21=barcodeData
            if left==503 and top==643:
                color22=barcodeData
            color10="green"                                    
            color00="red"
            # draw the barcode data and barcode type on the image
            text = "{}({}{})".format(barcodeData,left,top)
            cv2.putText(image, text, (x, y - 10), cv2.FONT_HERSHEY_SIMPLEX,
                        0.5, (0, 0, 255), 2)
            resized_image = cv2.resize(image, (720/2, 1280/2)) 
            # print the barcode type and data to the terminal
            #print("[INFO] Found {} barcode: {} with left{}top{} ".format(barcodeType, barcodeData,left,top))

        # show the output image
        #cv2.imshow("Image", image)    
        cv2.waitKey(3)






    # This function will be called when Action Server receives a Goal
    def on_goal(self, goal_handle):
        goal = goal_handle.get_goal()

        rospy.loginfo("Received new goal from Client")
        rospy.loginfo(goal)

        # Validate incoming goal parameters
        if((goal.protocol == "mqtt") or (goal.protocol=="http")):
            
            if((goal.mode == "pub") or (goal.mode == "sub")):
                goal_handle.set_accepted()
                
                # Start a new thread to process new goal from the client (For Asynchronous Processing of Goals)
                # 'self.process_goal' - is the function pointer which points to a function that will process incoming Goals
                thread = threading.Thread(  name="worker",
                                            target=self.process_goal,
                                            args=(goal_handle,) )
                thread.start()

            else:
                goal_handle.set_rejected()
                return
        
        else:
            goal_handle.set_rejected()
            return
    # This function will be called when Goal Cancel request is send to the Action Server
    def on_cancel(self, goal_handle):
        rospy.loginfo("Received cancel request.")
        goal_id = goal_handle.get_goal_id()



    def clear_octomap(self):
        clear_octomap_service_proxy = rospy.ServiceProxy(self._robot_ns + "/clear_octomap", Empty)
        return clear_octomap_service_proxy()

    def set_joint_angles(self, arg_list_joint_angles):

        list_joint_values = self._group.get_current_joint_values()
        # rospy.loginfo('\033[94m' + ">>> Current Joint Values:" + '\033[0m')
        # rospy.loginfo(list_joint_values)

        self._group.set_joint_value_target(arg_list_joint_angles)
        self._computed_plan = self._group.plan()
        flag_plan = self._group.go(wait=True)

        list_joint_values = self._group.get_current_joint_values()
        # rospy.loginfo('\033[94m' + ">>> Final Joint Values:" + '\033[0m')
        # rospy.loginfo(list_joint_values)

        pose_values = self._group.get_current_pose().pose
        # rospy.loginfo('\033[94m' + ">>> Final Pose:" + '\033[0m')
        # rospy.loginfo(pose_values)

        if (flag_plan == True):
            pass
            # rospy.loginfo(
            #     '\033[94m' + ">>> set_joint_angles() Success" + '\033[0m')
        else:
            pass
            # rospy.logerr(
            #     '\033[94m' + ">>> set_joint_angles() Failed." + '\033[0m')

        return flag_plan

    def hard_set_joint_angles(self, arg_list_joint_angles, arg_max_attempts):

        number_attempts = 0
        flag_success = False
        
        while ( (number_attempts <= arg_max_attempts) and  (flag_success is False) ):
            number_attempts += 1
            flag_success = self.set_joint_angles(arg_list_joint_angles)
            rospy.logwarn("attempts: {}".format(number_attempts) )
            # self.clear_octomap()


    def moveit_play_planned_path_from_file(self, arg_file_path, arg_file_name):
        file_path = arg_file_path + arg_file_name
        
        with open(file_path, 'r') as file_open:
            loaded_plan = yaml.load(file_open, Loader=yaml.FullLoader)
        
        ret = self._group.execute(loaded_plan)
        # rospy.logerr(ret)
        return ret
    def get_time_str(self,arg):
 
        
        print(datetime.today())  #print today's date time
        new_date = datetime.today() + timedelta(arg)
        print (new_date) #print new date time after addition of days to the current date
        
        if arg==0:
            return datetime.today()
        else:
            return new_date
        
    def moveit_hard_play_planned_path_from_file(self, arg_file_path, arg_file_name, arg_max_attempts):
        number_attempts = 0
        flag_success = False

        while ( (number_attempts <= arg_max_attempts) and (flag_success is False) ):
            number_attempts += 1
            flag_success = self.moveit_play_planned_path_from_file(arg_file_path, arg_file_name)
            rospy.logwarn("attempts: {}".format(number_attempts) )
            # # self.clear_octomap()
        
        return True
    def conveyor(self,powe):                                             #Function to set speed of conveyor belt

        rospy.wait_for_service('/eyrc/vb/conveyor/set_power')
        act_serv=rospy.ServiceProxy('/eyrc/vb/conveyor/set_power',conveyorBeltPowerMsg)
        cb=conveyorBeltPowerMsgRequest()
        cb.power=powe
        res=act_serv(cb)

    def set_planning_time(self, seconds):
        #Specify the amount of time to be used for motion planning. """
        self._group.set_planning_time(seconds)

    def gripper(self,on):                                             #Function to set speed of conveyor belt

        rospy.wait_for_service('/eyrc/vb/ur5/activate_vacuum_gripper/ur5_1')
        activate=rospy.ServiceProxy('/eyrc/vb/ur5/activate_vacuum_gripper/ur5_1',vacuumGripper)
        b=vacuumGripperRequest()
        b.activate_vacuum_gripper=on
        res=activate(bool(on))

    def att_a(self):                               # Function to attach box to vacuum_gripper in RViz
        box_name = self._box_name
        robot = self._robot
        scene = self._scene
        eef_link = self._eef_link
        group_names = self._group_names

        grasping_group = 'manipulator'
        touch_links = robot.get_link_names(group=grasping_group)
        scene.attach_box(eef_link, box_name, touch_links=touch_links)


    def func_callback_topic_my_topic(self,msg):                          #callback function for mqtt subscription messages 
        l= PriorityQueue()
        ur5 = Ur5Moveit(sys.argv[1])
        #rospy.loginfo("Dataaaaaaaaaaaaaaaaaaaaaaaaaaa %s ",str(msg.message))
        
        self.res = json.loads(str(msg.message)) 
        r=json.dumps(self.res)
        #print("The cooooooooooooooonverted dictionary : " + str(r))
        self.x= self.res["item"]
        print(self.x)
        self.order_id=self.res["order_id"]
        self.city=self.res["city"]
        self.qty=self.res["qty"]
        
        if self.x=="Medicine":
            self.queue.append(1)
            self._cost="450"
            self.prior="HP"
            self.city_queue.append(self.city)
            self.orderid_queue.append(self.order_id)
        #    l.insert(6)
        if self.x=="Food": 
            self.queue.append(2)   
            self._cost="250"
            self.prior="MP"
            self.city_queue.append(self.city)
            self.orderid_queue.append(self.order_id)            
        #    l.insert(9)
        if self.x=="Clothes":
            self.queue.append(3) 
            self._cost="100"   
            self.prior="LP"       
            self.city_queue.append(self.city)
            self.orderid_queue.append(self.order_id)        

        self.queue.sort(reverse=True)

        #if not self.already_called2:
        #print("hoooooooooooooooooooooooooooo")
        #ur5.sheetpush_dispatch(self.order_id,self.city,self.x,self.prior,self.qty,self._cost,"Dispatched","aa")
        ur5.sheetpush_incoming(self.res["order_id"],self.res["order_time"],self.res["item"],self.prior,self.res["qty"],self.res["city"],self.res["lon"],self.res["lat"],self._cost)
        #self.already_called2 = True 

        #print(self.queue)
        """
        while self.queue:
            queue_item=self.queue.pop()
            print(queue_item)
        #    print(l.get())
        #    print(l.queue)
            break
        """
        #    l.insert(5)
        #print(l.delete())
                             
        """
        try: 
            ur5 = Ur5Moveit(sys.argv[1]) 
            l= PriorityQueue()
            rospy.Subscriber("/ros_iot_bridge/mqtt/sub", msgMqttSub,ur5.func_callback_topic_my_topic)        
            rospy.logwarn("***************************************************************")
            #print(l.queue)
            if (l.delete()==1):
                print("got it") 
                ur5.high_priority()
            if (l.delete()==2):
                print("got it 2") 
                ur5.mid_priority()
            if (l.delete()==3):
                print("got it 3") 
        
                ur5.low_priority()        
        except:
            print("waiting")        
        
        try:

            if l.delete()==1:
                ur5.high_priority()
                print("got it")
             
            if l.delete()==3:
                ur5.low_priority("green")
            if l.delete()==2:
                ur5.mid_priority("yellow")
            
        except:
            print("waiting") 
        """

    def go_to_00(argv):
        ur5 = Ur5Moveit(sys.argv[1])



                                
        lst_joint_angles_00 = [math.radians(-51.278),
                                math.radians(-61.19739789),
                                math.radians(-6.14931),
                                math.radians(-113.4379),
                                math.radians(-128.7686),
                                math.radians(-0.529531156)]  
        ur5.hard_set_joint_angles(lst_joint_angles_00, 5)
        
        ur5.gripper("true")
        ur5.moveit_hard_play_planned_path_from_file(ur5._file_path, '00_to_belt.yaml', 5)
        ur5.gripper("")
        ur5.conveyor(50)
        #ur5.sheetpush_dispatch(self.res["order_id"],self.res["city"],self.res["item"],self.prior,self.res["qty"],self._cost,"Dispatched",self.res["lat"])

    def go_to_12(argv):
        lst_joint_angles_12 = [math.radians(60.9172),
                            math.radians(-93.3516),
                            math.radians(-43.103867),
                            math.radians(136.443684),
                            math.radians(-122.693609595),
                            math.radians(-0.0228205)]    
        ur5 = Ur5Moveit(sys.argv[1])
        ur5.hard_set_joint_angles(lst_joint_angles_12, 15)
        
        ur5.moveit_hard_play_planned_path_from_file(ur5._file_path, 'belt_to_12.yaml', 15)
        ur5.gripper("true")
        ur5.moveit_hard_play_planned_path_from_file(ur5._file_path, '12_to_belt.yaml', 5)
        ur5.gripper("")
        ur5.conveyor(50)    
    def go_to_02(argv):
        ur5 = Ur5Moveit(sys.argv[1])
        ur5.moveit_hard_play_planned_path_from_file(ur5._file_path, 'belt_to_02.yaml', 5)
        ur5.gripper("true")
        ur5.moveit_hard_play_planned_path_from_file(ur5._file_path, '02_to_belt.yaml', 5)    
        ur5.gripper("")
        ur5.conveyor(50)
        #ur5.sheetpush_dispatch(self.res["order_id"],self.res["city"],self.res["item"],self.prior,self.res["qty"],self._cost,"Dispatched",self.res["lat"])
    def go_to_01(argv):
        ur5 = Ur5Moveit(sys.argv[1])
        lst_joint_angles_01 = [math.radians(124.9195),
                                math.radians(-96.904712),
                                math.radians(-12.2235),
                                math.radians(-70.8793),
                                math.radians(51.3937),
                                math.radians(-179.974391)]        


        ur5.hard_set_joint_angles(lst_joint_angles_01, 5)
        #ur5.moveit_hard_play_planned_path_from_file(ur5._file_path, 'belt@to@01.yaml', 5)
        ur5.gripper("true")
        ur5.moveit_hard_play_planned_path_from_file(ur5._file_path, '01@to@belt.yaml', 5)
        ur5.gripper("")
        rospy.loginfo("#######")
        ur5.conveyor(50)
    def go_to_10(argv):
        ur5 = Ur5Moveit(sys.argv[1])
        ur5.moveit_hard_play_planned_path_from_file(ur5._file_path, 'belt_to_10.yaml', 5)
        ur5.gripper("true")
        ur5.moveit_hard_play_planned_path_from_file(ur5._file_path, '10_to_belt.yaml', 5)
        ur5.gripper("")
        ur5.conveyor(50)    
    def go_to_11(argv):
        ur5 = Ur5Moveit(sys.argv[1])
        ur5.moveit_hard_play_planned_path_from_file(ur5._file_path, 'belt_to_11.yaml', 5)
        ur5.gripper("true")
        ur5.moveit_hard_play_planned_path_from_file(ur5._file_path, '11_to_belt.yaml', 5)
        ur5.gripper("") 
        ur5.conveyor(50)           
    def go_to_20(argv):
        ur5 = Ur5Moveit(sys.argv[1])
        ur5.moveit_hard_play_planned_path_from_file(ur5._file_path, 'belt$to$20.yaml', 5)
        ur5.gripper("true")
        ur5.moveit_hard_play_planned_path_from_file(ur5._file_path, '20$to$belt.yaml', 5)
        ur5.gripper("")
        ur5.conveyor(50)        
    def go_to_21(argv):
        lst_joint_angles_21 = [math.radians(118.343029224),
                                    math.radians(-63.8670063072),
                                    math.radians(-103.093839949),
                                    math.radians(164.393303608),
                                    math.radians(-61.7077766538),
                                    math.radians(-178.782840069)]        
        ur5 = Ur5Moveit(sys.argv[1])
        ur5.hard_set_joint_angles(lst_joint_angles_21, 15)
        #ur5.moveit_hard_play_planned_path_from_file(ur5._file_path, 'belt$to$21.yaml', 15)
        ur5.gripper("true")
        ur5.moveit_hard_play_planned_path_from_file(ur5._file_path, '21$to$belt.yaml', 15)
        ur5.gripper("")  
        ur5.conveyor(50)      
    def go_to_22(argv): 
        ur5 = Ur5Moveit(sys.argv[1])
        ur5.moveit_hard_play_planned_path_from_file(ur5._file_path, 'belt$to$22.yaml', 5)
        ur5.gripper("true")
        ur5.moveit_hard_play_planned_path_from_file(ur5._file_path, '22$to$belt.yaml', 5)
        ur5.gripper("")      
        ur5.conveyor(50) 
    def __del__(self):
        moveit_commander.roscpp_shutdown()
        rospy.loginfo(
            '\033[94m' + "Object of class Ur5Moveit Deleted." + '\033[0m')
    def high_priority(arg):
        ur5 = Ur5Moveit(sys.argv[1])
        global color00,color01,color02,color10,color11,color12,color20,color21,color22
        global a,h,i
        #color00="red"
        while True:
            if color00=="red" and a==1:
                #if not self.already_called1:arg_orderid,city,arg_item,arg_prior,arg_quantity,arg_cost,arg_status,disp_date_time
                ur5.go_to_00()
                #ur5.sheetpush_dispatch(self.res["order_id"],self.res["city"],self.res["item"],self.prior,self.res["qty"],self._cost,"Dispatched",self.res["lat"])
                #self.already_called1=True
                rospy.logwarn("hoooooooooooooooooooooooooooo")
                a=0
                print(a)
                #ur5.sheetpush_dispatch(self.order_id,self.city,self.x,self.prior,self.qty,self._cost,"Dispatched","aa")
                break
            if color01=="red":
                ur5.go_to_01()
                break
            if color02=="red":
                ur5.go_to_02()
                break
            if color10=="red":
                ur5.go_to_10()
                break
            if color11=="red":
                ur5.go_to_11()
                break
            if color12=="red" and h==1:
                #if not self.already_called2:
                ur5.go_to_12()
                #self.already_called1=True
                h=0
                break
            if color20=="red":
                ur5.go_to_20()
                break
            if color21=="red" and i==1 :
                #if not self.already_called3:
                ur5.go_to_21()
                #self.already_called3=True
                i=0
                break
            if color22=="red":
                ur5.go_to_22()
                break

    def mid_priority(arg):
        ur5 = Ur5Moveit(sys.argv[1])
        global color00,color01,color02,color10,color11,color12,color20,color21,color22
        global e,f,g
        #color00="red"
        while True:
            if color00=="yellow":
                ur5.go_to_00()
                break
            if color01=="yellow" and e==1:
                #if not self.already_called4:
                ur5.go_to_01()
                #self.already_called4=True
                e=0
                print(e)
                break
            if color02=="yellow":
                ur5.go_to_02()
                break
            if color10=="yellow":
                ur5.go_to_10()
                break
            if color11=="yellow" and f==1 :
                #if not self.already_called5:
                ur5.go_to_11()
                #self.already_called5=True
                f=0
                break
            if color12=="yellow":
                ur5.go_to_12()
                break
            if color20=="yellow":
                ur5.go_to_20()
                break
            if color21=="yellow":
                ur5.go_to_21()
                break
            if color22=="yellow" and g==1:
                #if not self.already_called6:
                ur5.go_to_22()
                #self.already_called6=True
                g=0
                break

    def low_priority(arg):
        ur5 = Ur5Moveit(sys.argv[1])
        global color00,color01,color02,color10,color11,color12,color20,color21,color22
        global b,c,d
        #color00="red"
        while True:
            if color00=="green" :
                ur5.go_to_00()
                break
            if color01=="green":
                ur5.go_to_01()
                break
            if color02=="green" and b==1:
                rospy.logwarn("********bas shru hone")
                #if not self.already_called7:
                    
                ur5.go_to_02()
                #self.already_called7=True
                #ur5.sheetpush_dispatch(self.res["order_id"],self.res["city"],self.res["item"],self.prior,self.res["qty"],self._cost,"Dispatched",self.res["lat"])
                b=0
                print(b)
                rospy.logwarn("hoooooooooooooooooooooooooooo")
                #ur5.sheetpush_dispatch(self.order_id,self.city,self.x,self.prior,self.qty,self._cost,"Dispatched","aa")
                break
            if color10=="green" and d==1 :
                #if not self.already_called8:
                ur5.go_to_10()
                #self.already_called8=True
                d=0
                print(d)
                break
            if color11=="green":
                ur5.go_to_11()
                break
            if color12=="green":
                ur5.go_to_12()
                break
            if color20=="green" and c==1:
                #if not self.already_called9:
                ur5.go_to_20()
                #self.already_called9=True
                c=0
                break
            if color21=="green":
                ur5.go_to_21()
                break
            if color22=="green":
                ur5.go_to_22()
                break


def main():



    rospy.logwarn("********bas shru hone vala hai*********************************************************")       
    #print(l.queue)
    rospy.logwarn("********bas shru hone vala hai MCCCC***************************************************") 
    #while not l.isEmpty(): 
    #    print(l.delete())
     
    lst_joint_angles_belt = [math.radians(157.279830),             #joint angles for different boxes in shelf and for the belt
                            math.radians(-42.44022),
                            math.radians(57.4528),
                            math.radians(-109.6249),
                            math.radians(-86.6964),
                            math.radians(-114.21736)] 
    lst_joint_angles_belt2 = [math.radians(-0.122713),
                            math.radians(-140.975435),
                            math.radians(-42.02229219),
                            math.radians(-82.8198),
                            math.radians(93.5278),
                            math.radians(-91.74129)]


                            




    lst_joint_angles_00 = [math.radians(-51.278),
                            math.radians(-61.19739789),
                            math.radians(-6.14931),
                            math.radians(-113.4379),
                            math.radians(-128.7686),
                            math.radians(-0.529531156)]    
    lst_joint_angles_02= [math.radians(56.575044127),
                            math.radians(-122.340346),
                            math.radians(18.0023),
                            math.radians(-74.34453),
                            math.radians(124.71767),
                            math.radians(-86.31290)]
    lst_joint_angles_10 = [math.radians(-50.8090516056),
                            math.radians(-94.319002411),
                            math.radians(82.0668271095),
                            math.radians(-169.11699),
                            math.radians(-127.873469),
                            math.radians(-87.8985035646)]
    lst_joint_angles_11 = [math.radians(-122),
                            math.radians(-101.79344),
                            math.radians(55.0856864),
                            math.radians(45.085686),
                            math.radians(56.302596),
                            math.radians(67.4538408242)]
    lst_joint_angles_12 = [math.radians(60.9172),
                            math.radians(-93.3516),
                            math.radians(-43.103867),
                            math.radians(136.443684),
                            math.radians(-122.693609595),
                            math.radians(-0.0228205)]
    lst_joint_angles_20 = [math.radians(-47.986562),
                            math.radians(-93.400821),
                            math.radians(83.64549),
                            math.radians(9.7529),
                            math.radians(132.016063),
                            math.radians(72.96947)]
    lst_joint_angles_21 = [math.radians(120.91246782),
                            math.radians(-61.126257),
                            math.radians(-129.014921),
                            math.radians(10.14516),
                            math.radians(61.987488),
                            math.radians(179.99779)]

    """                      
    lst_joint_angles_22 = [math.radians(58.6323),
                            math.radians(-105.129577),
                            math.radians(-134.8164),
                            math.radians(59.9250),
                            math.radians(121.1490),
                            math.radians(179.9864)]
    """

    lst_joint_angles_30 = [math.radians(-52.56683),
                            math.radians(-89.359833),
                            math.radians(115.570428),
                            math.radians(-26.209250),
                            math.radians(124.5357),
                            math.radians(-0.005870)]
    lst_joint_angles_01 = [math.radians(124.9195),
                            math.radians(-96.904712),
                            math.radians(-12.2235),
                            math.radians(-70.8793),
                            math.radians(51.3937),
                            math.radians(-179.974391)]
    """lst_joint_angles_32 = [math.radians(-160.949),
                            math.radians(-94.7209),
                            math.radians(122.0628),
                            math.radians(-27.3422),
                            math.radians(18.2593),
    
                            math.radians(-0.013961)]

    """                        




    lst_joint_angles_20 = [math.radians(-55.890592036),
                                math.radians(-96.190635144),
                                math.radians(88.2680134448),
                                math.radians(7.93256921542),
                                math.radians(121.583594671),
                                math.radians(0.0140460466474)]
    lst_joint_angles_21 = [math.radians(118.343029224),
                                math.radians(-63.8670063072),
                                math.radians(-103.093839949),
                                math.radians(164.393303608),
                                math.radians(-61.7077766538),
                                math.radians(-178.782840069)]
    lst_joint_angles_22 = [math.radians(56.8630480846),
                                math.radians(-83.3737184212),
                                math.radians(-88.967325057),
                                math.radians(169.680178542),
                                math.radians(-117.474535602),
                                math.radians(-174.159593496)]
    #rospy.sleep(15)
    
    ur5 = Ur5Moveit(sys.argv[1]) 
    
    ur5.get_time_str(5)
    rospy.Subscriber("/ros_iot_bridge/mqtt/sub", msgMqttSub,ur5.func_callback_topic_my_topic,queue_size=1, buff_size = 2**28)        
    while not rospy.is_shutdown():
    #    try:
    
        
        #while l:    
        rospy.logwarn("111111111111111111111111")
        
        #r = rospy.rate(5)
        
        if ur5.queue==[]:
            rospy.logwarn("bhut jldi hori hai")
            #rospy.sleep(5)


        #rospy.logwarn("88888888888888888888888888888888888888888") 
        try :
            print(ur5.queue)
            queue_item=ur5.queue.pop()
            print(ur5.queue)
            if (queue_item==1):
                print("got it") 
                ur5.high_priority()
                #ur5.sheetpush_dispatch(self.order_id,self.city,self.x,self.prior,self.qty,self._cost,"Dispatched","aa")
                ur5.sheetpush_dispatch(ur5.orderid_queue.pop(0),ur5.city_queue.pop(0),"Medicines","HP","1","450","Dispatched",str(ur5.get_time_str(1)))
            if (queue_item==2):
                print("got it 2") 
                ur5.hard_set_joint_angles(lst_joint_angles_belt, 5)
                ur5.mid_priority()  
                ur5.sheetpush_dispatch(ur5.orderid_queue.pop(0),ur5.city_queue.pop(0),"Food","MP","1","250","Dispatched",str(ur5.get_time_str(3)))
            if (queue_item==3):
                print("got it 3") 
                ur5.hard_set_joint_angles(lst_joint_angles_belt, 5)
                ur5.low_priority()
                ur5.sheetpush_dispatch(ur5.orderid_queue.pop(0),ur5.city_queue.pop(0),"Clothes","LP","1","100","Dispatched",str(ur5.get_time_str(5))) 
            #if a==0 :    
            #    ur5.sheetpush_dispatch("1","","","","","","Dispatched","aa")
            #    a=2
            #if b==0 :    
            #    ur5.sheetpush_dispatch("2","","","","","","Dispatched","aa")
            #    b=2
            #if h==0 :    
            #    ur5.sheetpush_dispatch("3","","","","","","Dispatched","aa")  
            #    h=2


        except:
            print("ruko jra sbr kro")
            rospy.sleep(1)
        #rospy.logwarn("22222222222222222222222222")          
        #except:
        #    print("waiting") 
        #    rospy.sleep(2)

    
    
    """
    a=1
    b=1
    p=1
    c=1

    while True:
        if ur5.x=="Medicine" :
            if a==1:
                ur5.go_to_00()
                a=0
                break
            if b==1:
                ur5.go_to_01()
                b=0    
        elif ur5.x=="Food":
            if c==1:
                ur5.go_to_02()
                c=0
        elif ur5.x=="Clothes":
            if p==1:
                ur5.go_to_12()
                p=0
        else:
            pass

    """        

    #ur5.hard_set_joint_angles(lst_joint_angles_20, 5)
    #ur5.hard_set_joint_angles(lst_joint_angles_belt, 5)
    #ur5.moveit_hard_play_planned_path_from_file(ur5._file_path, '22$to$belt.yaml', 5)


    
    
    #ur5.set_planning_time(10)
    #while True:
    rospy.logwarn("33333333333333333333333333333333333333333333333")
        
        #rospy.loginfo("starting process")

    """

    ur5.hard_set_joint_angles(lst_joint_angles_00, 5)
    
    ur5.gripper("true")
    ur5.moveit_hard_play_planned_path_from_file(ur5._file_path, '00_to_belt.yaml', 5)
    ur5.gripper("")
    ur5.conveyor(50)

    ur5.moveit_hard_play_planned_path_from_file(ur5._file_path, 'belt_to_12.yaml', 5)
    ur5.gripper("true")
    ur5.moveit_hard_play_planned_path_from_file(ur5._file_path, '12_to_belt.yaml', 5)
    ur5.gripper("")
    ur5.conveyor(50)


    rospy.loginfo("########")


    ur5.moveit_hard_play_planned_path_from_file(ur5._file_path, 'belt$to$21.yaml', 5)
    ur5.gripper("true")
    ur5.moveit_hard_play_planned_path_from_file(ur5._file_path, '21$to$belt.yaml', 5)
    ur5.gripper("")
    ur5.conveyor(50)

    
    
    ur5.hard_set_joint_angles(lst_joint_angles_01, 5)
    #ur5.moveit_hard_play_planned_path_from_file(ur5._file_path, 'belt@to@01.yaml', 5)
    ur5.gripper("true")
    ur5.moveit_hard_play_planned_path_from_file(ur5._file_path, '01@to@belt.yaml', 5)
    ur5.gripper("")
    rospy.loginfo("#######")
    ur5.conveyor(50)



    
    rospy.loginfo("########")
    ur5.moveit_hard_play_planned_path_from_file(ur5._file_path, 'belt_to_10.yaml', 5)
    ur5.gripper("true")
    ur5.moveit_hard_play_planned_path_from_file(ur5._file_path, '10_to_belt.yaml', 5)
    ur5.gripper("")
    ur5.conveyor(50)


    rospy.loginfo("########")
    ur5.moveit_hard_play_planned_path_from_file(ur5._file_path, 'belt$to$22.yaml', 5)
    ur5.gripper("true")
    ur5.moveit_hard_play_planned_path_from_file(ur5._file_path, '22$to$belt.yaml', 5)
    ur5.gripper("")
    ur5.conveyor(50)

    


    ur5.moveit_hard_play_planned_path_from_file(ur5._file_path, 'belt_to_02.yaml', 5)
    ur5.gripper("true")
    ur5.moveit_hard_play_planned_path_from_file(ur5._file_path, '02_to_belt.yaml', 5)    
    ur5.gripper("")


    ur5.moveit_hard_play_planned_path_from_file(ur5._file_path, 'belt_to_11.yaml', 5)
    ur5.gripper("true")
    ur5.moveit_hard_play_planned_path_from_file(ur5._file_path, '11_to_belt.yaml', 5)
    ur5.gripper("")

      


    rospy.loginfo("########")
    ur5.moveit_hard_play_planned_path_from_file(ur5._file_path, 'belt$to$20.yaml', 5)
    ur5.gripper("true")
    ur5.moveit_hard_play_planned_path_from_file(ur5._file_path, '20$to$belt.yaml', 5)
    ur5.gripper("")
    ur5.conveyor(50)
    rospy.loginfo("########")
    """
    



    rospy.spin()

   


    



if __name__ == '__main__':
    main()

