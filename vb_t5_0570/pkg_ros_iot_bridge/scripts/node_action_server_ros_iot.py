#!/usr/bin/env python

# ROS Node - Action Server - IoT ROS Bridge

import rospy
import actionlib
import threading
import requests

 
from pkg_ros_iot_bridge.msg import msgIotRosAction      # Message Class that is used by ROS Actions internally
from pkg_ros_iot_bridge.msg import msgIotRosGoal        # Message Class that is used for Goal Messages
from pkg_ros_iot_bridge.msg import msgIotRosResult      # Message Class that is used for Result Messages
from pkg_ros_iot_bridge.msg import msgIotRosFeedback    # Message Class that is used for Feedback Messages    

from pkg_ros_iot_bridge.msg import msgMqttSub           # Message Class for MQTT Subscription Messages
from pkg_task1.msg import myActionMsgResult
import json
from pyiot import iot                                   # Custom Python Module to perfrom MQTT Tasks


class IotRosBridgeActionServer:


    # Constructor
    def __init__(self):
        # Initialize the Action Server
        self._as = actionlib.ActionServer('/action_iot_ros',
                                          msgIotRosAction,
                                          self.on_goal,
                                          self.on_cancel,
                                          auto_start=False)
    
    
 
    
        '''
            * self.on_goal - It is the fuction pointer which points to a function which will be called
                             when the Action Server receives a Goal.

            * self.on_cancel - It is the fuction pointer which points to a function which will be called
                             when the Action Server receives a Cancel Request.
        '''

        # Read and Store IoT Configuration data from Parameter Server
        param_config_iot = rospy.get_param('config_iot')
        self._config_mqtt_server_url = param_config_iot['mqtt']['server_url']
        self._config_mqtt_server_port = param_config_iot['mqtt']['server_port']
        self._config_mqtt_sub_topic = param_config_iot['mqtt']['topic_sub']
        self._config_mqtt_pub_topic = param_config_iot['mqtt']['topic_pub']
        self._config_mqtt_qos = param_config_iot['mqtt']['qos']
        self._config_mqtt_sub_cb_ros_topic = param_config_iot['mqtt']['sub_cb_ros_topic']
        print(param_config_iot)


        # Initialize ROS Topic Publication
        # Incoming message from MQTT Subscription will be published on a ROS Topic (/ros_iot_bridge/mqtt/sub).
        # ROS Nodes can subscribe to this ROS Topic (/ros_iot_bridge/mqtt/sub) to get messages from MQTT Subscription.
        self._handle_ros_pub = rospy.Publisher(self._config_mqtt_sub_cb_ros_topic, msgMqttSub, queue_size=10)


        # Subscribe to MQTT Topic (eyrc/xYzqLm/iot_to_ros) which is defined in 'config_iot_ros.yaml'.
        # self.mqtt_sub_callback() function will be called when there is a message from MQTT Subscription.
        ret = iot.mqtt_subscribe_thread_start(  self.mqtt_sub_callback, 
                                                        self._config_mqtt_server_url, 
                                                        self._config_mqtt_server_port, 
                                                        self._config_mqtt_sub_topic, 
                                                        self._config_mqtt_qos   )
        if(ret == 0):
            rospy.loginfo("MQTT Subscribe Thread Started")
        else:
            rospy.logerr("Failed to start MQTT Subscribe Thread")


        # Start the Action Server
        self._as.start()
        
        rospy.loginfo("Started ROS-IoT Bridge Action Server.")

    #callback function for client pose subcriptions 
    def func_callback_topic(self, pose_message):
        self._curr_x = pose_message.x
        self._curr_y = pose_message.y
        self._curr_theta = pose_message.theta

    
    # This is a callback function for MQTT Subscriptions
    def mqtt_sub_callback(self, client, userdata, message):
        payload = str(message.payload.decode("utf-8"))
    
        print("[MQTT SUB CB] Message: ", payload)
        print("[MQTT SUB CB] Topic: ", message.topic)

        msg_mqtt_sub = msgMqttSub()
        msg_mqtt_sub.timestamp = rospy.Time.now()
        msg_mqtt_sub.topic = message.topic
        msg_mqtt_sub.message = payload
        
        self._handle_ros_pub.publish(msg_mqtt_sub)
    
    
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


    # This function is called is a separate thread to process Goal.
    def process_goal(self, goal_handle):

        flag_success = False
        result = msgIotRosResult()

        goal_id = goal_handle.get_goal_id()
        rospy.loginfo("Processing goal : " + str(goal_id.id))

        goal = goal_handle.get_goal()
        URL2 = "https://script.google.com/macros/s/AKfycbym7KxAx364MVHRnvSSS6DgM-UfwOZPwHnyFSQq5rWa6nrwK_LW8pw6XQ/exec"#x y theta
        URL1="https://script.google.com/macros/s/AKfycbw850dk4moVgebU2GGe0PUQUvvg8jTpSjBQCawJt3_13vgujLk/exec"
        URL="https://script.google.com/macros/s/AKfycbym7KxAx364MVHRnvSSS6DgM-UfwOZPwHnyFSQq5rWa6nrwK_LW8pw6XQ/exec"# inventory
        URL3="https://script.google.com/macros/s/AKfycbym7KxAx364MVHRnvSSS6DgM-UfwOZPwHnyFSQq5rWa6nrwK_LW8pw6XQ/exec"


        
        # Goal Processing
        if(goal.protocol == "mqtt"):
            rospy.logwarn("MQTT")

            if(goal.mode == "pub"):
                rospy.logwarn("MQTT PUB Goal ID: " + str(goal_id.id))

                rospy.logwarn(goal.topic + " > " + goal.message)

                ret = iot.mqtt_publish( self._config_mqtt_server_url, 
                                        self._config_mqtt_server_port,
                                        goal.topic, 
                                        goal.message, 
                                        self._config_mqtt_qos   )

                if(ret == 0):
                    rospy.loginfo("MQTT Publish Successful.")
                    result.flag_success = True
                else:
                    rospy.logerr("MQTT Failed to Publish")
                    result.flag_success = False


            elif(goal.mode == "sub"):
                rospy.logwarn("MQTT SUB Goal ID: " + str(goal_id.id))
                rospy.logwarn(goal.topic)

                ret = iot.mqtt_subscribe_thread_start(  self.mqtt_sub_callback, 
                                                        self._config_mqtt_server_url, 
                                                        self._config_mqtt_server_port, 
                                                        goal.topic, 
                                                        self._config_mqtt_qos   )
                if(ret == 0):
                    rospy.loginfo("MQTT Subscribe Thread Started")
                    result.flag_success = True
                else:
                    rospy.logerr("Failed to start MQTT Subscribe Thread")
                    result.flag_success = False

            rospy.loginfo("Send goal result to client")



        rospy.loginfo("Goal ID: " + str(goal_id.id) + " Goal Processing Done.")

        #condition to send goal to sheet

        if(goal.protocol == "http"):
            rospy.logwarn("http")
            

            if(goal.mode == "pub"):
                rospy.logwarn("http PUB Goal ID: " + str(goal_id.id))                #here we update sheets using .get() requests
                res = json.loads(str(goal.message)) 
                r=json.dumps(res)
                rospy.logwarn(goal.topic + " > " + str(r))

				#print("The cooooooooooooooonverted dictionary : " + str(r))
				
				
                #parameters1 = {'id':'Sheet1','x':goal.message[0],'y':goal.message[2],'theta':goal.message[4]+goal.message[5]}
                #parameters = {"id":"Inventory", "Team Id":"0570", "Unique Id":"kanishka", "SKU":res["SKU"], "Item":res["Item"], "Priority":res["Priority"], "Storage Number":res["Storage Number"] , "Cost":res["Cost"]   ,"Quantity":res["Quantity"]} 
                #parameters2 = {"id":"IncomingOrders", "Team Id":"0570", "Unique Id":"kanishka", "Order ID": res["Order ID"],"Order Date and Time": res["Order Time"], "Item":res["Item"], "Priority":res["Priority"], "Order Quantity":res["Order Quantity"] ,"City":res["City"],"Longitude":res["Longitude"],"Latitude":res["Latitude"],"Cost":res["Cost"] } 
                parameters3 = {"id":"OrdersDispatched", "Team Id":"0570", "Unique Id":"kanishka", "Order ID": res["Order ID"],"City":res["City"], "Item":res["Item"], "Priority":res["Priority"], "Dispatch Quantity":res["Dispatch Quantity"],"Cost":res["Cost"] ,"Dispatch Status":res["Dispatch Status"],"Dispatch Date and Time":res["Dispatch Date and Time"] }
                #parameters4 = {"id":"OrdersShipped", "Team Id":"0570", "Unique Id":"kanishka", "Order ID": res["Order ID"],"City":res["City"], "Item":res["Item"], "Priority":res["Priority"], "Shipped Quantity":res["Shipped Quantity"],"Cost":res["Cost"] ,"Shipped Status":res["Shipped Status"],"Shipped Date and Time":res["Shipped Date and Time"],"Estimated Time of Delivery":res["Estimated Time of Delivery"] }
                response = requests.get(url= URL3, params=parameters3)
                print(response.content)


            rospy.loginfo("Goal ID: " + str(goal_id.id) + " Goal Processing Done.")

    
    # This function will be called when Goal Cancel request is send to the Action Server
    def on_cancel(self, goal_handle):
        rospy.loginfo("Received cancel request.")
        goal_id = goal_handle.get_goal_id()

    def func_callback_topic_my_topic(self,msg):                          #callback function for mqtt subscription messages 
        
        rospy.loginfo("Data Received %s ",str(msg.message))


# Main
def main():
    rospy.init_node('node_action_server_ros_iot_bridge')

    action_server = IotRosBridgeActionServer()

    rospy.Subscriber("/eyrc/vb/kanishka/orders",msgMqttSub, action_server.func_callback_topic_my_topic)


    rospy.spin()



if __name__ == '__main__':
    main()
