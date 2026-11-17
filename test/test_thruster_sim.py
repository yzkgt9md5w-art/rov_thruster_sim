import time
import pytest
import rclpy
from geometry_msgs.msg import Twist
from rov_thruster_sim.thruster_bridge_node import RovThrusterSimNode


@pytest.fixture(scope='module')
def ros_context():
    rclpy.init()
    yield
    rclpy.shutdown()


def test_surge_mixing_and_saturation(ros_context):
    node = RovThrusterSimNode()
    msg = Twist()
    msg.linear.x = 1.0  # Full forward surge

    node.cmd_vel_callback(msg)

    # All 4 horizontal thrusters should target +max_rpm; 2 vertical thrusters stay 0
    assert node.target_rpms[:4] == pytest.approx([node.max_rpm] * 4)
    assert node.target_rpms[4:] == pytest.approx([0.0, 0.0])
    node.destroy_node()


def test_yaw_differential_mixing(ros_context):
    node = RovThrusterSimNode()
    msg = Twist()
    msg.angular.z = 0.5  # Counter-clockwise yaw

    node.cmd_vel_callback(msg)

    # Left thrusters (indices 0, 2) negative; Right thrusters (indices 1, 3) positive
    assert node.target_rpms[0] == pytest.approx(-0.5 * node.max_rpm)
    assert node.target_rpms[2] == pytest.approx(-0.5 * node.max_rpm)
    assert node.target_rpms[1] == pytest.approx(0.5 * node.max_rpm)
    assert node.target_rpms[3] == pytest.approx(0.5 * node.max_rpm)
    node.destroy_node()


def test_first_order_motor_lag_response(ros_context):
    node = RovThrusterSimNode()
    node.noise_std = 0.0  # Disable noise for deterministic step check

    msg = Twist()
    msg.linear.x = 1.0
    node.cmd_vel_callback(msg)

    # Run control loop for 3 * tau seconds (~95% step response convergence)
    steps = int((3.0 * node.tau) / node.dt)
    for _ in range(steps):
        node.last_cmd_time = node.get_clock().now()  # Keep watchdog alive
        node.control_loop()

    expected_min = 0.92 * node.max_rpm
    assert node.current_rpms[0] >= expected_min
    assert node.current_rpms[0] <= node.max_rpm
    node.destroy_node()


def test_firmware_watchdog_failsafe(ros_context):
    node = RovThrusterSimNode()
    msg = Twist()
    msg.linear.x = 1.0
    node.cmd_vel_callback(msg)
    assert node.target_rpms[0] == pytest.approx(node.max_rpm)

    # Simulate timeout exceeding cmd_timeout
    time.sleep(node.cmd_timeout + 0.05)
    node.control_loop()

    assert node.target_rpms == [0.0] * 6
    node.destroy_node()


def test_pwm_deadband_and_bounds(ros_context):
    node = RovThrusterSimNode()
    assert node.rpm_to_pwm(0.0) == 1500
    assert node.rpm_to_pwm(10.0) == 1500  # Inside deadband
    assert node.rpm_to_pwm(node.max_rpm) == 1900
    assert node.rpm_to_pwm(-node.max_rpm) == 1100
    node.destroy_node()