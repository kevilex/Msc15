import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Joy
from geometry_msgs.msg import TwistStamped
from control_msgs.msg import JointJog
from std_srvs.srv import Trigger
from moveit_msgs.msg import PlanningScene, CollisionObject
import threading
import time


class JoyToServoPub(Node):
    def __init__(self):
        super().__init__('joy_to_twist_publisher')
        self.frame_to_publish = 'base_link'

        # Setup pub/sub
        self.joy_sub = self.create_subscription(
            Joy, '/joy', self.joy_callback, 10)
        self.twist_pub = self.create_publisher(
            TwistStamped, '/servo_node/delta_twist_cmds', 10)
        self.joint_pub = self.create_publisher(
            JointJog, '/servo_node/delta_joint_cmds', 10)
        self.collision_pub = self.create_publisher(
            PlanningScene, '/planning_scene', 10)
        
        # Create a service client to start the ServoNode
        #self.servo_start_client = self.create_client(Trigger, '/servo_node/start_servo')
        #while not self.servo_start_client.wait_for_service(timeout_sec=1.0):
        #    self.get_logger().info('service not available, waiting again...')
        request = Trigger.Request()
        #future = self.servo_start_client.call_async(request)

        # Load the collision scene asynchronously
        self.collision_pub_thread = threading.Thread(target=self.load_collision_scene)
        self.collision_pub_thread.start()
        self.AXIS_DEFAULTS = {5: 1.0, 2: 1.0}


    def joy_callback(self, msg):
        # Create the messages we might publish
        twist_msg = TwistStamped()
        joint_msg = JointJog()

        # This call updates the frame for twist commands
        self.update_cmd_frame(msg.buttons)

        # Convert the joystick message to Twist or JointJog and publish
        if self.convert_joy_to_cmd(msg.axes, msg.buttons, twist_msg, joint_msg):
            # publish the TwistStamped
            twist_msg.header.frame_id = self.frame_to_publish
            twist_msg.header.stamp = self.get_clock().now().to_msg()
            self.twist_pub.publish(twist_msg)
        else:
            # publish the JointJog
            joint_msg.header.stamp = self.get_clock().now().to_msg()
            joint_msg.header.frame_id = 'forearm_link'
            self.joint_pub.publish(joint_msg)

    def convert_joy_to_cmd(self, axes, buttons, twist, joint):
        # Give joint jogging priority because it is only buttons
        # If any joint jog command is requested, we are only publishing joint commands
        if buttons[0] or buttons[1] or buttons[2] or buttons[3] or axes[6] or axes[7]:
            # Map the D_PAD to the proximal joints
            joint.joint_names.extend(["base_link", "forearm_link"])
            joint.velocities.extend([axes[6], axes[7]])

            # Map the diamond to the distal joints
            joint.joint_names.extend(["wrist_2_link", "wrist_1_link"])
            joint.velocities.extend([buttons[1] - buttons[2], buttons[3] - buttons[0]])
            return False

        # The bread and butter: map buttons to twist commands
        twist.twist.linear.z = axes[4]
        twist.twist.linear.y = axes[3]

        lin_x_right = -0.5 * (axes[5] - self.AXIS_DEFAULTS.get(5, 1.0))
        lin_x_left = 0.5 * (axes[2] - self.AXIS_DEFAULTS.get(2, 1.0))
        twist.twist.linear.x = lin_x_right + lin_x_left

        twist.twist.angular.y = axes[1]
        twist.twist.angular.x = axes[0]

        roll_positive = buttons[5]
        roll_negative = -1 * buttons[4]
        twist.twist.angular.z = float(roll_positive + roll_negative)

        return True
        
    def update_cmd_frame(self, buttons):
        if buttons[6] and self.frame_to_publish == 'tool0':
            self.frame_to_publish = 'base_link'
        elif buttons[7] and self.frame_to_publish == 'base_link':
            self.frame_to_publish = 'tool0'

    def load_collision_scene(self):
        time.sleep(3)
        # Create a PlanningScene message
        planning_scene = PlanningScene()
        planning_scene.is_diff = True

        # Create collision object, in the way of servoing
        collision_object = CollisionObject()
        collision_object.id = 'box'

        # ... (same collision scene setup as in C++ code)

        planning_scene.world.collision_objects.append(collision_object)

        self.collision_pub.publish(planning_scene)

    def destroy(self):
        if self.collision_pub_thread.is_alive():
            self.collision_pub_thread.join()


def main(args=None):
    rclpy.init(args=args)
    joy_to_servo_pub = JoyToServoPub()
    rclpy.spin(joy_to_servo_pub)
    joy_to_servo_pub.destroy()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
