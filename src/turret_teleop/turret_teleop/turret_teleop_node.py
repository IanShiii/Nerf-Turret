import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Joy
from std_msgs.msg import Float64
from std_srvs.srv import Trigger
from std_srvs.srv import SetBool


class TurretTeleopNode(Node):
    def __init__(self):
        super().__init__('turret_teleop_node')

        self.joy_subscriber = self.create_subscription(Joy, '/joy', self.joy_callback, 10)

        self.pan_angle_publisher = self.create_publisher(Float64, '/turret_controller/target_pan_angle', 10)
        self.tilt_angle_publisher = self.create_publisher(Float64, '/turret_controller/target_tilt_angle', 10)

        self.trigger_client = self.create_client(Trigger, '/turret_controller/trigger')
        self.set_flywheel_client = self.create_client(SetBool, '/turret_controller/set_flywheel')

        self.trigger_client.wait_for_service()
        self.set_flywheel_client.wait_for_service()

        self.get_logger().info('Turret Teleop Node has been started.')

    def joy_callback(self, msg: Joy):
        # Left joystick Y controls tilt
        tilt_angle = (msg.axes[1] + 1.0) / 2.0 * 30.0 + 90.0  # Map from [-1, 1] to [90, 120]
        tilt_msg = Float64()
        tilt_msg.data = tilt_angle
        self.tilt_angle_publisher.publish(tilt_msg)

        # Right joystick X controls pan
        pan_angle = (-msg.axes[2] + 1.0) / 2.0 * 180.0  # Map from [-1, 1] to [0, 180]
        pan_msg = Float64()
        pan_msg.data = pan_angle
        self.pan_angle_publisher.publish(pan_msg)

        # Right Trigger shoots
        if msg.buttons[7] == 1:
            self.trigger_client.call_async(Trigger.Request())

        # Top button enables flywheel
        if msg.buttons[3] == 1:
            flywheel_request = SetBool.Request()
            flywheel_request.data = True
            self.set_flywheel_client.call_async(flywheel_request)
        
        # Bottom button disables flywheel
        if msg.buttons[1] == 1:
            flywheel_request = SetBool.Request()
            flywheel_request.data = False
            self.set_flywheel_client.call_async(flywheel_request)



def main(args=None):
    rclpy.init(args=args)
    node = TurretTeleopNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
