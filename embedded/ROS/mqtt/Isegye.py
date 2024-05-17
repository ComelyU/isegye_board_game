import rclpy
from rclpy.node import Node
from rclpy.action import ActionClient
from geometry_msgs.msg import PoseStamped
from std_msgs.msg import String
from nav2_msgs.action import NavigateToPose
from dotenv import load_dotenv
import paho.mqtt.client as mqtt
import paho.mqtt.publish as mqtt_publish
import threading, time, os, json

# .env
load_dotenv(dotenv_path = '/home/jetson/.env')
BROKER_ADDRESS = os.environ.get("BROKER_ADDRESS")
PORT = int(os.environ.get("PORT"))
TURTLE_TOPIC = os.environ.get("TURTLE_TOPIC")
SERVER_TOPIC = os.environ.get("SERVER_TOPIC")
USER_NAME = os.environ.get("USER_NAME")
PASSWORD = os.environ.get("PASSWORD")
HOME_X = os.environ.get("HOME_X")
HOME_Y = os.environ.get("HOME_Y")


class IsegyeNode(Node):
    def __init__(self):
        super().__init__('isegye_node')
        self.get_logger().info('IsegyeNode start!')

        # mqtt json
        self.turtleId = 1
        self.turtleOrderLogId = None
        self.turtleReceiveLogId = None
        self.coordinateX = None
        self.coordinateY = None

        # Action Client
        self.client = ActionClient(self, NavigateToPose, '/navigate_to_pose')
        self.get_logger().info('Action client started...')
        self.goal_handle = None  # Initialize goal_handle variable

        # Qt
        self.button_sub = self.create_subscription(String, "/button_order", self.button_callback, 10)
        self.display_pub = self.create_publisher(String, "display", 10)
        self.first_next = False
        
        # turtlebot state
        self.turtlebot_state = "wait" # wait, wake
        self.is_home_order = False

        # MQTT
        self.sub_thread = threading.Thread(target = self.subscriber_thread)

        # thread start
        self.sub_thread.start()

    # when clicked button
    def button_callback(self, msg):
        self.get_logger().info(f'receive button_order from Qt with [{msg.data}]')
        time.sleep(1)
        self.publisher_thread()

        if not self.first_next:
            self.IoT_publisher_thread()
            self.first_next = True
    
    # send goal to navigation
    def send_goal(self, x ,y):
        self.get_logger().info("Start send_goal...")
        goal_req = NavigateToPose.Goal()
        goal_req.pose.header.frame_id = 'map'
        goal_req.pose.pose.position.x = x
        goal_req.pose.pose.position.y = y
        goal_req.pose.pose.orientation.w = 1.0  # Assuming no rotation is needed

        if self.client.wait_for_server(5) is False:
            self.get_logger().info('Service not available...')
            return
        
        goal_future = self.client.send_goal_async(goal_req, feedback_callback=self.feedback_callback)
        self.get_logger().info('send_goal_async done...')
        goal_future.add_done_callback(self.goal_callback)
    
    def goal_callback(self, future):
        self.get_logger().info('Start goal_callback...')
        self.goal_handle = future.result()

        if not self.goal_handle.accepted:
            self.get_logger().info('Goal rejected...')
            return

        self.get_logger().info('Goal accepted...')
        self._get_result_future = self.goal_handle.get_result_async()
        self._get_result_future.add_done_callback(self.get_result_callback)
    
    # arrival status
    def get_result_callback(self, future):
        self.goal_handle = None

        self.get_logger().info('Arrived at the destination!')
        msg = String()
        msg.data = "button"
        self.display_pub.publish(msg)
        self.get_logger().info("publish 'button' to display")

        if self.is_home_order:
            self.is_home_order = False
            msg.data = "sleep"
            self.display_pub.publish(msg)
            self.get_logger().info("publish 'sleep' to display")
            self.turtlebot_state = "wait"
            self.get_logger().info('Arrived at the Home! [Turtlebot State]: wait')
    
    def feedback_callback(self, msg):
        if self.goal_handle is None:
            # Goal handle not yet initialized
            return

        feedback = msg.feedback
        # self.get_logger().info(f'Received feedback: {feedback}')
        # self.get_logger().info(f'Remaining distance: {feedback.remaining_distance}')
    
    # cancel all_goal from nav_action_server
    def cancel_goal(self):
        self.get_logger().info("Start cancel_goal")

        if self.goal_handle is None:
            self.get_logger().info('No active goal to cancel...')
            return
    
        if not self.goal_handle.accepted:
            self.get_logger().info('Goal is not accepted yet. Cannot cancel...')
            return

        self.get_logger().info('Cancelling goal...')
        cancel_future = self.goal_handle.cancel_goal_async()
        cancel_future.add_done_callback(self.cancel_callback)
    
    def cancel_callback(self, future):
        result_handle = future.result()
        if len(result_handle.goals_canceling) > 0:
            self.get_logger().info('Goal cancelled successfully...')
        else:
            self.get_logger().info('Failed to cancel goal...')

    # connect callback
    def on_connect(self, client, userdata, flags, rc):
        client_name = str(client._client_id, 'utf-8')
        result_codes = {
            0: f"Success - The connection, [{client_name}] request has been successfully processed.",
            1: "Protocol Error - Incorrect protocol version or invalid protocol data was received.",
            2: "Client ID rejected - The client identifier was rejected by the server.",
            3: "Server unavailable - The server is currently unavailable.",
            4: "Bad username or password - Incorrect username or password was provided.",
            5: "Not authorized - The client is not authorized to perform the operation.",
            "6-255": "Reserved for future use - Various error codes currently reserved and may be defined later."
        }
        
        if rc == 0:
            self.get_logger().info(f"Connected to MQTT broker successfully with result code {rc}.")
            client.subscribe(TURTLE_TOPIC)
            self.get_logger().info(f"subscribe topic: [{TURTLE_TOPIC}]")
        
        else:
            self.get_logger().info(f"Failed to connect to MQTT broker with result code {rc}")

        self.get_logger().info(f"code {rc}: {result_codes[rc]}")
        
    # subscriber callback
    def on_message(self, client, userdata, message):
        try:
            mqtt_msg = str(message.payload.decode('utf-8'))
            json_data = json.loads(mqtt_msg)
            self.get_logger().info("Received MQTT message: " + mqtt_msg)

            self.turtleOrderLogId = json_data["turtleOrderLogId"]
            self.turtleReceiveLogId = json_data["turtleReceiveLogId"]
            self.coordinateX = float(json_data["coordinateX"])
            self.coordinateY = float(json_data["coordinateY"])

            self.get_logger().info(f"turtleOrderLogId: {self.turtleOrderLogId}, turtleReceiveLogId: {self.turtleReceiveLogId}, x: {self.coordinateX}, y: {self.coordinateY}")
           
            if self.turtlebot_state == "wait":
                msg = String()
                msg.data = "wake"
                self.display_pub.publish(msg)
                self.turtlebot_state = "wake"
                self.get_logger().info("publish turtlebot_state: wake")
            
            if self.coordinateX == float(HOME_X) and self.coordinateY == float(HOME_Y):
                self.is_home_order = True

            # send goal
            self.send_goal(self.coordinateX, self.coordinateY)
        
        except Exception as e:
            self.get_logger().error(f"Error while processing MQTT message: {e}")

    # subscriber thread
    def subscriber_thread(self):
        try:
            sub_client = mqtt.Client("sub_client")  #구독자 이름
            sub_client.on_connect = self.on_connect
            sub_client.on_message = self.on_message
            sub_client.username_pw_set(username=USER_NAME, password=PASSWORD)
            sub_client.connect(BROKER_ADDRESS, PORT, 60)  #broker 주소 등록
            sub_client.loop_forever()
        
        except Exception as e:
            self.get_logger().error(f"Error in MQTT subscriber thread: {e}")

    def publisher_thread(self):
        self.get_logger().info("start publishing...")

        json_data = {
                 "turtleId" : self.turtleId, 
                 "turtleOrderLogId" : self.turtleOrderLogId, 
                 "turtleReceiveLogId" : self.turtleReceiveLogId 
             }
        
        msg = json.dumps(json_data)
        
        try:
            mqtt_publish.single(SERVER_TOPIC, msg, qos=0, retain=False, hostname=BROKER_ADDRESS, port=PORT, auth={'username':USER_NAME, 'password':PASSWORD})
            self.get_logger().info("[pub] [topic]: " + SERVER_TOPIC + ", [msg]: " + msg)

        except Exception as e:
            self.get_logger().error(f"Error in MQTT publisher thread: {e}")
    
    def IoT_publisher_thread(self):
        self.get_logger().info("start IoT_publishing...")
        
        msg = "webview"
        
        try:
            mqtt_publish.single("display/1", msg, qos=0, retain=False, hostname=BROKER_ADDRESS, port=PORT, auth={'username':USER_NAME, 'password':PASSWORD})
            self.get_logger().info("[pub] [topic]: display/1, [msg]: " + msg)

        except Exception as e:
            self.get_logger().error(f"Error in MQTT IoT_publisher thread: {e}")

def main(args = None):
    rclpy.init(args = args)
    isegye_node = IsegyeNode()

    try:
        rclpy.spin(isegye_node)

    except KeyboardInterrupt:
        print("Input Ctrl + C... now destroy Node...")
        print("Please press 'Ctrl + C' again...")
        isegye_node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
