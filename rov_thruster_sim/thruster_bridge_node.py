import random
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from sensor_msgs.msg import JointState
from std_msgs.msg import Int32MultiArray


class RovThrusterSimNode(Node):
    def __init__(self):
        super().__init__('rov_thruster_sim_node')

        # Declare configurable firmware/thruster parameters
        self.declare_parameter('max_rpm', 3500.0)
        self.declare_parameter('time_constant', 0.15)      # Motor inertia + water drag lag (seconds)
        self.declare_parameter('noise_stddev', 8.0)        # Hall-effect / ESC telemetry noise (RPM)
        self.declare_parameter('cmd_timeout', 0.5)         # Failsafe watchdog timeout (seconds)
        self.declare_parameter('sim_rate_hz', 50.0)        # Control loop frequency

        self.max_rpm = self.get_parameter('max_rpm').value
        self.tau = self.get_parameter('time_constant').value
        self.noise_std = self.get_parameter('noise_stddev').value
        self.cmd_timeout = self.get_parameter('cmd_timeout').value
        self.rate_hz = self.get_parameter('sim_rate_hz').value
        self.dt = 1.0 / self.rate_hz

        # 6-Thruster ROV Layout: [Front-Left, Front-Right, Rear-Left, Rear-Right, Vert-Left, Vert-Right]
        self.thruster_names = [
            'thruster_fl', 'thruster_fr',
            'thruster_rl', 'thruster_rr',
            'thruster_vl', 'thruster_vr'
        ]
        self.num_thrusters = len(self.thruster_names)

        self.target_rpms = [0.0] * self.num_thrusters
        self.current_rpms = [0.0] * self.num_thrusters
        self.last_cmd_time = self.get_clock().now()

        # ROS 2 Interfaces
        self.cmd_sub = self.create_subscription(
            Twist, '/cmd_vel', self.cmd_vel_callback, 10
        )
        self.telemetry_pub = self.create_publisher(
            JointState, '/thrusters/rpm_telemetry', 10
        )
        self.pwm_pub = self.create_publisher(
            Int32MultiArray, '/thrusters/pwm_debug', 10
        )

        self.timer = self.create_timer(self.dt, self.control_loop)
        self.get_logger().info('ROV Thruster Firmware Simulator & ROS 2 Bridge initialized.')

    def cmd_vel_callback(self, msg: Twist):
        self.last_cmd_time = self.get_clock().now()

        # Normalized surge (x), sway (y), heave (z), yaw (angular.z)
        vx = msg.linear.x
        vy = msg.linear.y
        vz = msg.linear.z
        wz = msg.angular.z

        # 45-degree vectored horizontal mixing + 2 vertical thrusters
        raw_mix = [
            vx - vy - wz,   # Front-Left
            vx + vy + wz,   # Front-Right
            vx + vy - wz,   # Rear-Left
            vx - vy + wz,   # Rear-Right
            vz,             # Vertical-Left
            vz              # Vertical-Right
        ]

        # Normalize if any channel exceeds 1.0 to preserve steering authority
        max_magnitude = max(1.0, max(abs(val) for val in raw_mix))
        self.target_rpms = [
            (val / max_magnitude) * self.max_rpm for val in raw_mix
        ]

    def rpm_to_pwm(self, rpm: float) -> int:
        # Map [-max_rpm, +max_rpm] to standard ESC PWM [1100, 1900] us with deadband
        if abs(rpm) < 25.0:
            return 1500
        normalized = max(-1.0, min(1.0, rpm / self.max_rpm))
        deadband = 25 if normalized > 0 else -25
        return int(1500 + deadband + normalized * (400 - abs(deadband)))

    def control_loop(self):
        now = self.get_clock().now()
        elapsed = (now - self.last_cmd_time).nanoseconds * 1e-9

        # Firmware Watchdog: Zero targets if cmd_vel stream stops
        if elapsed > self.cmd_timeout:
            self.target_rpms = [0.0] * self.num_thrusters

        # First-order motor response filter factor
        alpha = self.dt / (self.tau + self.dt)

        measured_rpms = []
        pwm_signals = []

        for i in range(self.num_thrusters):
            # Simulate physical rotor acceleration / hydrodynamic drag lag
            self.current_rpms[i] += alpha * (self.target_rpms[i] - self.current_rpms[i])

            # Add realistic telemetry sensor noise when spinning
            noise = random.gauss(0.0, self.noise_std) if abs(self.current_rpms[i]) > 1.0 else 0.0
            sensed_rpm = max(-self.max_rpm, min(self.max_rpm, self.current_rpms[i] + noise))

            measured_rpms.append(sensed_rpm)
            pwm_signals.append(self.rpm_to_pwm(self.current_rpms[i]))

        # Publish RPM Telemetry
        telemetry_msg = JointState()
        telemetry_msg.header.stamp = now.to_msg()
        telemetry_msg.name = self.thruster_names
        telemetry_msg.velocity = measured_rpms  # Reported in RPM
        self.telemetry_pub.publish(telemetry_msg)

        # Publish simulated ESC PWM output
        pwm_msg = Int32MultiArray()
        pwm_msg.data = pwm_signals
        self.pwm_pub.publish(pwm_msg)


def main(args=None):
    rclpy.init(args=args)
    node = RovThrusterSimNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()