import rclpy, tf2_ros
from rclpy.time import Time
from rclpy.qos import QoSProfile
from rclpy.node import Node
from sensor_msgs.msg import Joy, PointCloud2, PointField
from geometry_msgs.msg import TwistStamped, TransformStamped
from control_msgs.msg import JointJog
from std_srvs.srv import Trigger
from moveit_msgs.msg import PlanningScene, CollisionObject
import pyrealsense2 as rs
import threading, ros2_numpy, numpy as np, array, time
import tf2_py as tf2
from tf2_ros import LookupException, ConnectivityException, ExtrapolationException
#from tf2_ros import quaternion, translation
#from tf.transformations.quaternion_matrix

class JoyToServoPub(Node):
    def __init__(self):
        super().__init__('joy_to_twist_publisher')
        self.frame_to_publish = 'tool0'

        # Setup pub/sub
        self.joy_sub = self.create_subscription(
            Joy, '/joy', self.joy_callback, 10)
        self.twist_pub = self.create_publisher(
            TwistStamped, '/servo_node/delta_twist_cmds', 10)
        self.joint_pub = self.create_publisher(
            JointJog, '/servo_node/delta_joint_cmds', 10)
        self.collision_pub = self.create_publisher(
            PlanningScene, '/planning_scene', 10)
        self.pc_pub = self.create_publisher(
            PointCloud2, 'camera/depth', 10)
        
        #tf 
        self.tf_buffer = tf2_ros.Buffer()
        self.tf_listener = tf2_ros.TransformListener(self.tf_buffer, self)
        
        #camera
        self.pipe = rs.pipeline()
        self.cfg = rs.config()
        self.pc = rs.pointcloud()
        self.profile = self.pipe.start(self.cfg)
        self.temporal = rs.temporal_filter()
        self.decimation = rs.decimation_filter(magnitude = 5)

        self.msg = PointCloud2()
        self.points = np.array([], dtype=[('f0', '<f4'), ('f1', '<f4'), ('f2', '<f4')])
        self.msgCount = 1

        # Load the collision scene asynchronously
        self.collision_pub_thread = threading.Thread(target=self.load_collision_scene)
        self.collision_pub_thread.start()
        self.AXIS_DEFAULTS = {5: 1.0, 2: 1.0}

        print('Playstation mode initiated')
        print('--------------------------------')
        print('Cross: Take picture and betwixtnt saving')
        print('Circle: Save combined pointcloud')
        print('Square: Reset Scan')


    def joy_callback(self, msg):
        # Create the messages we might publish
        twist_msg = TwistStamped()
        joint_msg = JointJog()

        # This call updates the frame for twist commands
        self.update_cmd_frame(msg.axes)

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

        if buttons[0]:
            self.publish_pc()
            print('picture taken, saved as intermittent_points' + str(self.msgCount) + '.ply')
        if buttons[1]:
            self.save_points_as_ply(self.points, 'combined.ply')
            print('combined picture saved as combined.ply')
        if buttons[2]:
            self.points = np.array([], dtype=[('f0', '<f4'), ('f1', '<f4'), ('f2', '<f4')])
            print('reset points')

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
        
    def update_cmd_frame(self, axes):
        if (axes[7] < -0.1) and self.frame_to_publish == 'tool0':
            self.frame_to_publish = 'base_link'
        elif (axes[7] > 0.1) and self.frame_to_publish == 'base_link':
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


    def take_pic(self):
        frames = []
        for x in range(10):
            frameset = self.pipe.wait_for_frames()
            frames.append(frameset.get_depth_frame())
        
        for x in range(10):
            temp_filtered = self.temporal.process(frames[x])
        
        decimated_depth = self.decimation.process(temp_filtered)

        points = self.pc.calculate(decimated_depth)
        return points

    def publish_pc(self):

        #retrieve filtered pointcloud
        points = self.take_pic()
        points = np.asanyarray(points.get_vertices(), dtype=[('f0', '<f4'), ('f1', '<f4'), ('f2', '<f4')])


        #transform
        transform = self.get_transform('wrist_3_link', 'base_link')
        if transform is None:
            print('No transform')
            return  # or handle the error as appropriate
        trans_matrix = self.quaternion_to_matrix(transform)
        transformed_points_list = []

        for id, point in enumerate(points):
            # Apply the transformation
            transformed_point = self.transform_point(point, trans_matrix)

            transformed_points_list.append((transformed_point['f0'], transformed_point['f1'], transformed_point['f2']))
        transformed_points_array = np.array(transformed_points_list, dtype=self.points.dtype)

        self.save_points_as_ply(transformed_points_array, ('intermittent_points' + str(self.msgCount) + '.ply'))


        #add transformed points to the total points array
        self.points = np.concatenate((self.points, transformed_points_array))


        #create the pointcloud message        
        self.msg.header.frame_id = 'base_link'
        current_time = time.time()
        self.msg.header.stamp.sec = int(current_time)
        self.msg.header.stamp.nanosec = int((current_time - self.msg.header.stamp.sec) * 1e9)
        self.msg.height = 1
        self.msg.width = len(points) * self.msgCount
        self.msg.point_step = 12
        self.msg.row_step = self.msg.point_step * self.msg.width

        self.msg.fields = [
            PointField(name='x', offset=0, datatype=PointField.FLOAT32, count=1),
            PointField(name='y', offset=4, datatype=PointField.FLOAT32, count=1),
            PointField(name='z', offset=8, datatype=PointField.FLOAT32, count=1)]
        self.msg.is_bigendian = False
        self.msg.is_dense = False



        # Flatten the data
        flattened = np.hstack((self.points['f0'].reshape(-1, 1), self.points['f1'].reshape(-1, 1), self.points['f2'].reshape(-1, 1)))
        self.msgCount = self.msgCount + 1
        
        # Add the flattened data to the message
        self.msg.data = np.asarray(flattened, np.float32).tobytes()

        self.pc_pub.publish(self.msg)


    def save_points_as_ply(self, points, filename):
        with open(filename, 'w') as ply_file:
            # Write PLY header
            ply_file.write("ply\n")
            ply_file.write("format ascii 1.0\n")
            ply_file.write(f"element vertex {len(points)}\n")
            ply_file.write("property float x\n")
            ply_file.write("property float y\n")
            ply_file.write("property float z\n")
            ply_file.write("end_header\n")

            # Write point data
            for point in points:
                ply_file.write(f"{point['f0']} {point['f1']} {point['f2']}\n")


    #tf attempt
    def quaternion_to_matrix(self, transform):
        x, y, z, w = transform.rotation.x, transform.rotation.y, transform.rotation.z, transform.rotation.w
        xx, yy, zz, xy, xz, yz, wx, wy, wz = (x * x, y * y, z * z, x * y, x * z, y * z, w * x, w * y, w * z)

        matrix = np.array([
            [1 - 2 * (yy + zz),     2 * (xy - wz),     2 * (xz + wy), 0],
            [    2 * (xy + wz), 1 - 2 * (xx + zz),     2 * (yz - wx), 0],
            [    2 * (xz - wy),     2 * (yz + wx), 1 - 2 * (xx + yy), 0],
            [                0,                 0,                 0, 1]
        ])

        # Add the translation part
        matrix[:3, 3] = [transform.translation.x, transform.translation.y, transform.translation.z]

        return matrix
    

    def transform_point(self, point, matrix):
        point_homogeneous = np.array([point['f0'], point['f1'], point['f2'], 1])
        transformed_point = np.dot(matrix, point_homogeneous)
        return {'f0': transformed_point[0], 'f1': transformed_point[1], 'f2': transformed_point[2]}


    def get_transform(self, from_frame, to_frame):
        try:
            now = rclpy.time.Time()
            trans = self.tf_buffer.lookup_transform(to_frame, from_frame, now)
            return trans.transform
        except (LookupException, ConnectivityException, ExtrapolationException) as ex:
            self.get_logger().error('Could not transform %s to %s: %s' % (from_frame, to_frame, ex))
            return None
               
    #tf attempt
    
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
